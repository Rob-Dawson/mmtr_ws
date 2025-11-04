TODO and Problems

Currently the magnetometer and IMU are fused using the madgwick filter. This output (imu/data) goes into an zero_yaw.py in an attempt to zero the yaw so as yaw 0 is along the "Red" X axis in gazebo.
When checking the output, the yaw drifts over time 
    orientation:
      x: 0.0
      y: 0.0
      z: -0.022753672968859027
      w: 0.9997411016690402

Whereas imu/data states
    orientation:
      x: -5.26360914027985e-05
      y: 0.009860198284320063
      z: -0.019327017240055692
      w: 0.9997645923535391

imu/out I.e. Ground Truth is:
    orientation:
      x: 0.00019930561015695658
      y: -0.0001144842099999715
      z: 1.0883235235369964e-06
      w: 0.9999999735847285

After 2 hours    
    orientation:
      x: 0.0
      y: 0.0
      z: -0.02486018606865354
      w: 0.9996909378145987
  covariance:

This can probably be improved. Try removing the imu/data_zero_yaw from the UKF and instead rotate the world perhaps.
Check the covariances.

Also look at the stationary checks. These may downplay the covariances under motion and don't currently have a moving average. 

Zero the world, not the IMU:
Practical plan
Feed the UKF /imu/data (Madgwick) directly.

At startup, read yaw ψ₀ from /imu/data and publish map→odom = Rz(-ψ₀) once (or set the estimator’s initial pose).

Keep mag fused in Madgwick; in the UKF fuse orientation only and keep yaw Q small.