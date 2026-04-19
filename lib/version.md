Version 1
    - Get clean GPS lat/lon
    - Get BNO055 yaw/quaternion
    - Convert GPS to local x/y meters
    - Log everything to Serial
Version 2
    - Add IMU-based short-term velocity/position estimate
    - Blend IMU prediction with GPS correction
Version 3
    - 2D EKF in local ENU (east/north position and velocity) with BNO055 linear accel + heading in the predict step and GPS position updates
    - Origin policy aligned with V2 (default West Lafayette until a valid GPS fix, then snap)
    - CSV logging: sensor/EKF samples every 100 ms, one averaged row every 1000 ms (vector mean for headings via sin/cos + atan2)