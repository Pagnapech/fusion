Version 1
    - Get clean GPS lat/lon
    - Get BNO055 yaw/quaternion
    - Convert GPS to local x/y meters
    - Log everything to Serial
Version 2
    - Add IMU-based short-term velocity/position estimate
    - Blend IMU prediction with GPS correction
Version 3
    - Implement a simple 2D EKF