# Include directories
include_directories(${KINETIC_PATH}/inc)

# Kinetic library (core C code)
add_library(kinetic STATIC ${KINETIC_PATH}/src/kin_error.c ${KINETIC_PATH}/src/kin_math.c ${KINETIC_PATH}/src/kin_ekf.c ${KINETIC_PATH}/src/kin_imu.c)
target_include_directories(kinetic PUBLIC ${KINETIC_PATH}/kinetic/inc)
target_link_libraries(kinetic PRIVATE m)

