project(CFE_IMU_APP C)

include_directories(fsw/mission_inc)
include_directories(fsw/platform_inc)

# Create the app module
add_cfe_app(imu_app fsw/src/imu_app.c)

# Include the public API from mpu6050
# to call library-provided functions
add_cfe_app_dependency(imu_app mpu6050)
