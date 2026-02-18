#!/usr/bin/env python3
import math
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
from rclpy.time import Time
from mmtr_msg.msg import CollisionEvent
from mmtr_msg.msg import RewindEvent
SIDE_VEC = {0:(1,0), 1:(0,1), 2:(0,-1), 3:(-1,0)}  # FRONT, LEFT, RIGHT, REAR

PRIOR_PROB = 0.4
OCCUPANCY_PROB = 0.85  ## This will be handeled by the crash detector but for now,
## a shim could be useful to prove it works
FREE_PROB = 0.45


def prob_2_log_odds(probability):
    return math.log(probability / (1 - probability))


def log_odds_2_prob(log_odds):
    try:
        return 1.0 / (1.0 + math.exp(-log_odds))
    except OverflowError:
        return 1.0 if log_odds > 0 else 0.0


LOG_PRIOR_PROB = prob_2_log_odds(PRIOR_PROB)
LOG_OCCUPANCY_PROB = prob_2_log_odds(OCCUPANCY_PROB)
LOG_FREE_PROB = prob_2_log_odds(FREE_PROB)
LOG_PROB_MIN = -5.0
LOG_PROB_MAX = 5.0
PROB_LOCK = 0.65
LOG_LOCK = prob_2_log_odds(PROB_LOCK)

VISUALISE_LOG_ODDS_IN_RVIZ = True


##These global values use the robot's own geometry to help target the correct cells
ROBOT_LENGTH = 0.4
ROBOT_WIDTH = 0.15
OUTER_OFFSET = 0.1
BOX_SIZE_M = 0.3

ROBOT_RADIUS = 0.5 * math.sqrt(ROBOT_LENGTH * ROBOT_LENGTH + ROBOT_WIDTH * ROBOT_WIDTH)


class MappingWithCollisions(Node):
    ## Add log odds
    def __init__(self):
        super().__init__("Mapping_With_Collision")
        
        ## Map configuration
        self.width = 200
        self.height = 200
        self.resolution = 0.05

        self.robot_X = 0.0
        self.robot_Y = 0.0
        self.map = OccupancyGrid()
        self.map.header.frame_id = "mmtr/odom"
        self.map.info.resolution = self.resolution
        self.map.info.height = self.height
        self.map.info.width = self.width
        self.map.info.origin.position.x = -(self.width * self.resolution / 2.0)
        self.map.info.origin.position.y = -(self.height * self.resolution / 2.0)
        self.map.info.origin.orientation.w = 1.0

        self.map_origin_x = self.map.info.origin.position.x
        self.map_origin_y = self.map.info.origin.position.y

        ## Find all the cells around the robot's current
        ## cellX and cellY using a circle as the shape
        self.circle_offsets = [] 
        radius_cells = int(math.ceil(ROBOT_RADIUS/self.resolution))
        self.circle_offset(radius_cells)

        ## Params for the crash and rollback event 
        self.t_crash = None
        self.pose_crash = None
        self.rollback_pending = False

        ## Publishers and Subscribers
        self.collision_detected_sub = self.create_subscription(CollisionEvent, "/collision/event", self.collision_detected, 10)
        self.rewind_event_sub = self.create_subscription(RewindEvent, "/rewind_event", self.rewind_event_cb, 10)
        self.odom_sub = self.create_subscription(
            Odometry, "/odometry/filtered_ukf", self.odom_callback, 10
        )
        self.map_pub = self.create_publisher(OccupancyGrid, "map", 10)
        self.timer = self.create_timer(1.0, self.publish_map)

        ## Building the probability map using numpy as it is significantly faster for accessing and manipulation
        ## Initialises log_odds probability map to 0.5
        self.log_odds = np.full(
            (self.map.info.height, self.map.info.width),
            LOG_PRIOR_PROB,
            dtype=np.float32,
        )
        self.previous_cell_x = None
        self.previous_cell_y = None

        ## This grid will store the timestamps when a cell was last edited
        ## This will allow the rewind to "rewind" anything after the timestamp
        self.last_visited_ns = np.zeros((self.map.info.height, self.map.info.width),
                                        dtype=np.uint64)

        self.box_half_cells = max(1, math.ceil((BOX_SIZE_M / 2.0) / self.resolution))

        self.robot_yaw = 0.0
        self.bump_block_until = 0.0


        if VISUALISE_LOG_ODDS_IN_RVIZ:
            self.map_prob_pub = self.create_publisher(OccupancyGrid, "map_prob", 10)
            self.visited = np.zeros(self.log_odds.shape, dtype=bool)
       

    ## Finds all the cells within the radius of the robot 
    def circle_offset(self, radius_cells:int):
        radius = radius_cells
        for x in range(-radius, radius+1):
            for y in range(-radius, radius+1):
                if x*x + y*y <= radius*radius:
                    self.circle_offsets.append((x,y))
            


    def rewind_event_cb(self, rewind:RewindEvent):
        self.pose_crash = rewind.pose_crash 
        self.t_crash = rewind.t_crash
        self.rollback_pending = True

    ## Used for carving free space
    def bresenham(self, current_x, current_y, prev_x, prev_y):
        
        dx = abs(current_x - prev_x)
        dy = abs(current_y - prev_y)
        if prev_x < current_x:
            step_x = 1
        else:
            step_x = -1
        if prev_y < current_y:
            step_y = 1
        else:
            step_y = -1

        error = dx - dy
        grid_cell_x, grid_cell_y = prev_x, prev_y
        while True:
            yield grid_cell_x, grid_cell_y

            if grid_cell_x == current_x and grid_cell_y == current_y:
                break

            double_error = 2 * error

            if double_error > -dy:
                error -= dy
                grid_cell_x += step_x

            if double_error < dx:
                error += dx
                grid_cell_y += step_y

    def fill_box(self, current_x, current_y, half_cells):
        current_x = int(math.floor((current_x - self.map_origin_x) / self.resolution))
        current_y = int(math.floor((current_y - self.map_origin_y) / self.resolution))
        hc = int(half_cells)
        i0, i1 = max(0, current_x - hc), min(self.width - 1, current_x + hc)
        j0, j1 = max(0, current_y - hc), min(self.height - 1, current_y + hc)
        for i in range(i0, i1 + 1):
            for j in range(j0, j1 + 1):
                yield i, j


    def collision_detected(self, msg):
        self.bump_block_until = self.get_clock().now().nanoseconds * 1e-9 + 0.5
        ux, uy = SIDE_VEC.get(msg.side, (1, 0))
        c, s = math.cos(self.robot_yaw), math.sin(self.robot_yaw)
        vx, vy = c*ux - s*uy, s*ux + c*uy
        shell = (ROBOT_WIDTH/2.0) if msg.side in (1, 2) else (ROBOT_LENGTH/2.0)
        offset_m = shell + OUTER_OFFSET
        
        current_x = self.robot_X + vx * offset_m
        current_y = self.robot_Y + vy * offset_m

        for i, j in self.fill_box(current_x, current_y, self.box_half_cells):
            self.mark_as_occupied(i,j,confidence=getattr(msg, "confidence", 1.0))

    def mark_as_free(self, current_cell_x, current_cell_y):
        now = self.get_clock().now().nanoseconds

        for dx, dy in self.circle_offsets:           
            circle_cell_x = current_cell_x + dx
            circle_cell_y = current_cell_y + dy
            
            if 0 <= circle_cell_x < self.width and 0 <= circle_cell_y < self.height:
                ## This ensures that occupied cells with a probability above {LOG_LOCK} are not overwritten
                if self.log_odds[circle_cell_y, circle_cell_x] > LOG_LOCK:
                    continue

                self.log_odds[circle_cell_y, circle_cell_x] = min(self.log_odds[circle_cell_y, circle_cell_x], LOG_PROB_MIN)
                self.last_visited_ns[circle_cell_y, circle_cell_x] = now
                
                if VISUALISE_LOG_ODDS_IN_RVIZ:
                    self.visited[circle_cell_y, circle_cell_x] = True

    def mark_as_occupied(self, current_cell_x, current_cell_y, confidence=1.0):
        if 0 <= current_cell_x < self.width and 0 <= current_cell_y < self.height:
            log_occupancy = LOG_OCCUPANCY_PROB * max(0.3, min(1.0, confidence))
            self.log_odds[current_cell_y, current_cell_x] = np.clip(
                self.log_odds[current_cell_y, current_cell_x] + log_occupancy,
                LOG_PROB_MIN,
                LOG_PROB_MAX,
            )
            if VISUALISE_LOG_ODDS_IN_RVIZ:
                self.visited[current_cell_y, current_cell_x] = True

    def odom_callback(self, msg):
        now_s = self.get_clock().now().nanoseconds * 1e-9

        ##Get Robot current position
        self.robot_X = msg.pose.pose.position.x
        self.robot_Y = msg.pose.pose.position.y
        qx, qy, qz, qw = (msg.pose.pose.orientation.x,
                        msg.pose.pose.orientation.y,
                        msg.pose.pose.orientation.z,
                        msg.pose.pose.orientation.w)
        ##Find the YAW      
        siny_cosp = 2*(qw*qz + qx*qy)
        cosy_cosp = 1 - 2*(qx*qx + qy*qy)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)

        ##Set grid cell to current position of the robot
        current_cell_x = math.floor(
            (self.robot_X - self.map_origin_x) / self.resolution
        )
        current_cell_y = math.floor(
            (self.robot_Y - self.map_origin_y) / self.resolution
        )

        ## Initialise or skip carving during a bump
        if self.previous_cell_x is None and self.previous_cell_y is None:
            self.mark_as_free(current_cell_x, current_cell_y)
            self.previous_cell_x = current_cell_x
            self.previous_cell_y = current_cell_y
            return

        if now_s < self.bump_block_until:
            # still update previous cell so we don't draw a huge line next tick
            self.previous_cell_x = current_cell_x
            self.previous_cell_y = current_cell_y
            return

        ## If the current and previoud cell have not changed, do not perform bresenham
        if current_cell_x == self.previous_cell_x and current_cell_y == self.previous_cell_y:#
            return
        ##Carve a free path
        for cell_x, cell_y in self.bresenham(
            current_cell_x, current_cell_y, self.previous_cell_x, self.previous_cell_y):
            self.mark_as_free(cell_x, cell_y)

        self.previous_cell_x = current_cell_x
        self.previous_cell_y = current_cell_y

    def publish_map(self):
        header_stamp = self.get_clock().now().to_msg()
        probability = 1.0 / (1.0 + np.exp(-self.log_odds))
        # print(probability)
        ##Contains all the cells where prob is > 0.7
        occupied_mask = probability > 0.7
        ##Contains all the cells where last_visited_ns is != 0
        if self.rollback_pending:
            t_crash_sec = self.t_crash.sec
            t_crash_ns = self.t_crash.nanosec
            t_crash_ns = t_crash_sec*int(1e9) + t_crash_ns

            cells_after_crash = self.last_visited_ns >= t_crash_ns
            self.rollback_pending = False
            stuff_happened = self.last_visited_ns > 0.0
            print(self.last_visited_ns[stuff_happened])
            print(f"CRASH {t_crash_ns}")
            self.last_visited_ns[cells_after_crash] = 0

            self.log_odds[cells_after_crash] = LOG_PRIOR_PROB
            if VISUALISE_LOG_ODDS_IN_RVIZ:
                self.visited[cells_after_crash] = False

        free_mask = (self.last_visited_ns != 0) & (~occupied_mask)
        grid = np.full(self.log_odds.shape, -1, dtype=np.int8)
        grid[free_mask] = 0
        grid[occupied_mask] = 100


        



        self.map.data = grid.ravel(order="C").tolist()
        self.map.header.stamp = header_stamp
        self.map.header.frame_id = "mmtr/odom"
        self.map_pub.publish(self.map)
        hist, bins = np.histogram(self.log_odds, bins=10)
        # print(list(zip(bins, hist)))


        if VISUALISE_LOG_ODDS_IN_RVIZ:
            prob_msg = OccupancyGrid()
            prob_msg.info = self.map.info
            prob_msg.header.stamp = header_stamp
            prob_msg.header.frame_id = "mmtr/odom"

            prob_grid = np.full(self.log_odds.shape, -1, dtype=np.int16)
            prob_vals = (probability * 100.0).astype(np.int16)
            mask = self.visited
            prob_grid[mask] = prob_vals[mask]
            # print(prob_vals)
            prob_msg.data = prob_grid.ravel(order="C").tolist()
            self.map_prob_pub.publish(prob_msg)


def main(args=None):
    rclpy.init(args=args)
    node = MappingWithCollisions()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
