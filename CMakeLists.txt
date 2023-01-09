project(CFE_IMU_APP C)

include_directories(fsw/mission_inc)
include_directories(fsw/platform_inc)
include_directories(${imu_app_MISSION_DIR}/fsw/platform_inc)

# Create the app module
add_cfe_app(imu_app fsw/src/imu_app.c)
