#!/usr/bin/env python3

"""
Performs mean bias filter
Publishers:
    imu_filtered : Mean Bias Filtered
    accel_gravity_removed_world : Gravity removed and in the world frame
    accel_gravity_removed_body : Gravity removed and in the body frame

Subscriptions:
    /imu : This is the raw imu without any filtering
"""

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from enum import Enum, auto
from geometry_msgs.msg import Vector3Stamped, TwistWithCovarianceStamped
from mmtr_calibration.low_pass_filter import LowPassFilter
import numpy as np


class State(Enum):
    INIT = auto()
    COLLECT_GYRO_DATA = auto()
    COLLECT_ACCEL_AND_ORIENTAION_DATA = auto()

    CALCULATE_GYRO_BIAS = auto()
    CALCULATE_ACCEL_BIAS = auto()

    PUBLISH_GYRO_BIAS = auto()
    PUBLISH_CALIBRATED_IMU = auto()

    RECALIBRATE = auto()


def _mean_vector(buffer) -> np.ndarray:
    mean = np.mean([vec for _, vec in buffer], axis=0)
    return mean


def _quat_to_world_frame(qx, qy, qz, qw) -> np.ndarray:
    q = np.array([qx, qy, qz, qw], dtype=float)
    q /= np.linalg.norm(q)
    x, y, z, w = q
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z

    return np.array(
        [
            [1 - 2 * (yy + zz), 2 * (xy - wz), 2 * (xz + wy)],
            [2 * (xy + wz), 1 - 2 * (xx + zz), 2 * (yz - wx)],
            [2 * (xz - wy), 2 * (yz + wx), 1 - 2 * (xx + yy)],
        ],
        dtype=float,
    )


def _get_message_time(imu: Imu):
    return imu.header.stamp.sec + imu.header.stamp.nanosec * 1e-9


class IMUCalibration(Node):

    def _setup_pubs(self):
        self.filtered_imu = self.create_publisher(Imu, "imu/data_raw", 100)
        self.accel_body_pub = self.create_publisher(
            Vector3Stamped, "/imu/accel_gravity_removed_body", 10
        )

        self.accel_world_pub = self.create_publisher(
            Vector3Stamped, "imu/accel_gravity_removed_world", 10
        )

    def _setup_subs(self):
        self.imu_sub = self.create_subscription(Imu, "/imu_raw", self.imu_cb, 200)

        ## For correct accel mean bias, an accurate gravity vector with orientation is needed
        self.processed_imu_sub = self.create_subscription(
            Imu, "/imu/data", self.processed_imu_cb, 200
        )

        self.zupt_sub = self.create_subscription(TwistWithCovarianceStamped, "zupt/twist", self.zupt_cb, 200)

    def _init_state(self):
        self.state = State.INIT

        self.gravity = None

        self.gyro_buffer = []
        self.accel_buffer = []
        self.orientation_buffer = []
        self.latest_orientation = None
        self.latest_orientation_time = None

        self.gyro_error = np.zeros(3)
        self.accel_error = np.zeros(3)

        self.start_time = None
        self.recieved_madgwick = False
        self.gravity_mag = 0.0
        self.gravity_mean = np.zeros(3, dtype=float)


        self.last_filter_time = None
        self.last_recalibration_time = None
        self.true_linear_accel_body = 0
        self.true_linear_accel_world = 0
        
        self.accel_signal_lpf = LowPassFilter(cutoff_hz=20.0, sample_rate_hz=200.0, dim=3)
        self.gyro_signal_lpf = LowPassFilter(cutoff_hz=30.0, sample_rate_hz=200.0, dim=3)
        
        self.gyro_bias_lpf = LowPassFilter(
            cutoff_hz=0.05,
            sample_rate_hz=240.0,
            dim=3
        )

        self.accel_bias_lpf = LowPassFilter(
            cutoff_hz=0.05,
            sample_rate_hz=240.0,
            dim=3
        )

        self.last_zupt_time = None
        self.zupt_active = False

    def __init__(self):
        super().__init__("IMU_Calibration")
        self._setup_subs()
        self._setup_pubs()
        self._init_state()

    ###
    ### This will need rewriting at some point as it is
    ### not robust and quite fragile.
    ### Make this state-based and use the IMU
    def is_zupt_active(self):
        if self.last_zupt_time is None:
            return False
        age = (self.get_clock().now() - self.last_zupt_time).nanoseconds * 1e-9
        return age < 0.05

    def zupt_cb(self, zupt: TwistWithCovarianceStamped):
        self.last_zupt_time = self.get_clock().now()

    ###

        ## THis function contains the orientation estimate provided
        # by the madgwick filter

    def processed_imu_cb(self, imu: Imu):
        self.latest_orientation = imu.orientation
        self.latest_orientation_time = _get_message_time(imu)
        self.recieved_madgwick = True

    def imu_cb(self, imu: Imu):
        current_time = _get_message_time(imu)
        if self.start_time is None:
            self.start_time = current_time
        self.process_state_machine(current_time, imu)

    def handle_init(self):
        self.state = State.COLLECT_GYRO_DATA
        self.get_logger().info("COLLECT_GYRO_DATA")

    def handle_collect_gyro_data(self, current_time, imu: Imu):
        self.gyro_buffer.append(
            (
                current_time,
                np.array(
                    [
                        imu.angular_velocity.x,
                        imu.angular_velocity.y,
                        imu.angular_velocity.z,
                    ]
                ),
            )
        )

        if current_time - self.start_time >= 5.0:
            self.state = State.CALCULATE_GYRO_BIAS
            self.get_logger().info("CALCULATE_GYRO_BIAS")

    def handel_collect_accel_data(self, current_time, imu: Imu):
        self.accel_buffer.append(
            (
                current_time,
                np.array(
                    [
                        imu.linear_acceleration.x,
                        imu.linear_acceleration.y,
                        imu.linear_acceleration.z,
                    ]
                ),
            )
        )

        if current_time - self.start_time >= 5.0:
            self.state = State.CALCULATE_ACCEL_BIAS
            self.get_logger().info("CALCULATE_ACCEL_BIAS")

    def handel_collect_orientation_data(self, current_time, imu: Imu.orientation):
        self.orientation_buffer.append(
            (
                current_time,
                np.array(
                    [
                        imu.x,
                        imu.y,
                        imu.z,
                        imu.w,
                    ]
                ),
            )
        )
    
    def handle_calculate_gyro_bias(self):
        gyro_mean = _mean_vector(self.gyro_buffer)
        self.gyro_error = gyro_mean
        self.gyro_buffer.clear()
        self.state = State.PUBLISH_GYRO_BIAS

    def handle_calculate_accel_bias(self):
        self.gravity_body_mean, self.gravity_mag = self.gravity_estimation(
            self.accel_buffer
        )
        quats = np.array([quat for _, quat in self.orientation_buffer])
        rotations = np.array([_quat_to_world_frame(*quat) for quat in quats])
        gravity_world = np.array([0.0, 0.0, self.gravity_mag])
        gravity_body = np.array([R.T @ gravity_world for R in rotations])
        accels = np.array([accel for _, accel in self.accel_buffer])

        bias_samples = accels - gravity_body

        self.accel_error = bias_samples.mean(axis=0)
        self.state = State.PUBLISH_CALIBRATED_IMU
        self.get_logger().info("PUBLISH_CALIBRATED_IMU")

    def handle_publish_IMU(self, imu: Imu):
        ##This function publishes raw accel
        # with gyro calibrated angular_vel
        gyro_filtered_imu = Imu()
        gyro_filtered_imu.header.stamp = imu.header.stamp
        gyro_filtered_imu.header.frame_id = "imu_link"

        gyro_raw = np.array(
            [
                imu.angular_velocity.x,
                imu.angular_velocity.y,
                imu.angular_velocity.z,
            ]
        )
        gyro_corrected = gyro_raw - self.gyro_error

        gyro_filtered_imu.linear_acceleration.x = imu.linear_acceleration.x
        gyro_filtered_imu.linear_acceleration.y = imu.linear_acceleration.y
        gyro_filtered_imu.linear_acceleration.z = imu.linear_acceleration.z
        gyro_filtered_imu.linear_acceleration_covariance = (
            imu.linear_acceleration_covariance
        )

        gyro_filtered_imu.angular_velocity.x = gyro_corrected[0]
        gyro_filtered_imu.angular_velocity.y = gyro_corrected[1]
        gyro_filtered_imu.angular_velocity.z = gyro_corrected[2]
        gyro_filtered_imu.angular_velocity_covariance = imu.angular_velocity_covariance
        self.filtered_imu.publish(gyro_filtered_imu)
        # self.filtered_imu.publish(gyro_filtered_imu)

    def handle_publish_calibrated_IMU(self, imu: Imu):
        current_time = _get_message_time(imu)

        dt = current_time - self.last_filter_time if self.last_filter_time else None
        self.last_filter_time = current_time

        gyro_raw = np.array(
            [
                imu.angular_velocity.x,
                imu.angular_velocity.y,
                imu.angular_velocity.z
            ]
        )
        gyro_corrected = gyro_raw - self.gyro_error

        accel_raw = np.array(
            [
                imu.linear_acceleration.x,
                imu.linear_acceleration.y,
                imu.linear_acceleration.z,
            ]
        )
        accel_corrected = accel_raw - self.accel_error
        
        if dt is not None and 0.0 < dt < 0.1:
            gyro_corrected = self.gyro_signal_lpf.update(gyro_corrected, dt)
            accel_corrected = self.accel_signal_lpf.update(accel_corrected, dt)
            
        filtered_imu = Imu()
        filtered_imu.header = imu.header
        filtered_imu.linear_acceleration.x = float(accel_corrected[0])
        filtered_imu.linear_acceleration.y = float(accel_corrected[1])
        filtered_imu.linear_acceleration.z = float(accel_corrected[2])
        filtered_imu.linear_acceleration_covariance = imu.linear_acceleration_covariance

        filtered_imu.angular_velocity.x = float(gyro_corrected[0])
        filtered_imu.angular_velocity.y = float(gyro_corrected[1])
        filtered_imu.angular_velocity.z = float(gyro_corrected[2])
        filtered_imu.angular_velocity_covariance = imu.angular_velocity_covariance

        self.filtered_imu.publish(filtered_imu)

    def handle_publish_accel_without_gravity(self, imu: Imu):
        raw_time = _get_message_time(imu)
        if self.latest_orientation is None:
            return
        orientation_age = raw_time - self.latest_orientation_time
        if abs(orientation_age) >0.03:
            self.get_logger().warn(f"Stale orientation age={orientation_age:.4f}s")
            return
        accel_raw = np.array(
            [
                imu.linear_acceleration.x,
                imu.linear_acceleration.y,
                imu.linear_acceleration.z,
            ]
        )

        accel_corrected = accel_raw - self.accel_error

        latest_orientation = self.latest_orientation

        orientation_raw = np.array(
            [
                latest_orientation.x,
                latest_orientation.y,
                latest_orientation.z,
                latest_orientation.w,
            ]
        )

        rotate = _quat_to_world_frame(*orientation_raw)
        gravity_world = np.array([0.0, 0.0, self.gravity_mag])
        gravity_body = rotate.T @ gravity_world

        true_accel_body = accel_corrected - gravity_body

        true_accel_world = rotate @ true_accel_body

        
        true_accel_body_vector = Vector3Stamped()
        true_accel_body_vector.header.stamp = imu.header.stamp
        true_accel_body_vector.vector.x = float(true_accel_body[0])
        true_accel_body_vector.vector.y = float(true_accel_body[1])
        true_accel_body_vector.vector.z = float(true_accel_body[2])

        true_accel_world_vector = Vector3Stamped()
        true_accel_world_vector.header.stamp = imu.header.stamp
        true_accel_world_vector.vector.x = float(true_accel_world[0])
        true_accel_world_vector.vector.y = float(true_accel_world[1])
        true_accel_world_vector.vector.z = float(true_accel_world[2])

        self.accel_body_pub.publish(true_accel_body_vector)
        self.accel_world_pub.publish(true_accel_world_vector)

    def handle_recalibrate(self, imu:Imu):
        current_time = _get_message_time(imu)
        dt = (
            current_time - self.last_recalibration_time
            if self.last_recalibration_time is not None
            else None
        )
        self.last_recalibration_time = current_time

        if dt is None or not (0.0 < dt < 0.1):
            return

        if self.latest_orientation is None:
            return
        
        gyro_raw = np.array([imu.angular_velocity.x,
                             imu.angular_velocity.y,
                             imu.angular_velocity.z,])
        self.gyro_error = self.gyro_bias_lpf.update(
            gyro_raw,
            dt
        )
        accel_raw = np.array([imu.linear_acceleration.x,
                             imu.linear_acceleration.y,
                             imu.linear_acceleration.z,])
        
        print(self.latest_orientation)
        quat = np.array([
            self.latest_orientation.x,
            self.latest_orientation.y,
            self.latest_orientation.z,
            self.latest_orientation.w,
        ])
        R = _quat_to_world_frame(*quat)
        
        gravity_world = np.array([0.0, 0.0, self.gravity_mag])
        gravity_body = R.T @ gravity_world
        accel_bias_sample = accel_raw - gravity_body
        self.get_logger().info(
            f"gravity_mag ={self.gravity_mag}")
        self.get_logger().info(
            f"gravity body={gravity_body}")
        
        self.get_logger().info(
            f"accel_bias_sample ={accel_bias_sample}")

        
        
        self.accel_error = self.accel_bias_lpf.update(
            accel_bias_sample,
            dt
        )
        self.get_logger().info(
            f"accel_error={self.accel_error}")

    def process_state_machine(self, current_time, imu: Imu):
        if self.state == State.INIT:
            self.handle_init()

        elif self.state == State.COLLECT_GYRO_DATA:
            self.handle_collect_gyro_data(current_time, imu)

        elif self.state == State.CALCULATE_GYRO_BIAS:
            self.handle_calculate_gyro_bias()

        elif self.state == State.PUBLISH_GYRO_BIAS:
            if self.latest_orientation is None:
                self.get_logger().info("PUBLISH_GYRO_BIAS -- No orientation", once=True)
                self.handle_publish_IMU(imu)
            else:
                self.get_logger().info("PUBLISH_GYRO_BIAS -- With Orientation", once=True)
                self.state = State.COLLECT_ACCEL_AND_ORIENTAION_DATA
                self.get_logger().info("COLLECT_ACCEL_AND_ORIENTAION_DATA")
                self.start_time = current_time

        elif self.state == State.COLLECT_ACCEL_AND_ORIENTAION_DATA:
            self.handel_collect_accel_data(current_time, imu)
            self.handel_collect_orientation_data(self.latest_orientation_time, self.latest_orientation)

        elif self.state == State.CALCULATE_ACCEL_BIAS:
            self.handle_calculate_accel_bias()

        elif self.state == State.PUBLISH_CALIBRATED_IMU:
            self.handle_publish_calibrated_IMU(imu)
            self.handle_publish_accel_without_gravity(imu)
            self.get_logger().info(f"{self.accel_error}")


            # if self.is_zupt_active():
                # self.state = State.RECALIBRATE
                # self.last_recalibration_time = None
        
        elif self.state == State.RECALIBRATE:
            self.get_logger().info("RECALIBRATION", once=True)

            self.handle_recalibrate(imu)

            self.handle_publish_calibrated_IMU(imu)
            self.handle_publish_accel_without_gravity(imu)

            if not self.is_zupt_active():
                self.state = State.PUBLISH_CALIBRATED_IMU



    def gravity_estimation(self, accel_buffer):
        accels = np.array([vec for _, vec in accel_buffer])
        gravity_body_mean = np.mean(accels, axis=0)
        gravity_mag = np.linalg.norm(gravity_body_mean)

        print(f"Gravity Vector: {gravity_body_mean}")
        print(f"Gravity magnitude: {gravity_mag}")
        return gravity_body_mean, gravity_mag


def main(args=None):
    rclpy.init(args=args)
    node = IMUCalibration()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
