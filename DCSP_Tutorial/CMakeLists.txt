cmake_minimum_required(VERSION 3.29)
project(DCSP C)

#set(CMAKE_CXX_STANDARD 20)
set(CMAKE_C_STANDARD 17)

# library dir
find_library(NIDAQmx_LIB
NAMES NIDAQmx
PATHS Library
)



file(GLOB_RECURSE SOURCE
        src/*.c
)

file(GLOB_RECURSE HEADER
        include/*.h
)



add_executable(DCSP_Tutorial main.c)
target_link_libraries(DCSP_Tutorial PUBLIC ${NIDAQmx_LIB})
target_include_directories(DCSP_Tutorial PUBLIC Library)

#==================================================================================================================
# Tutorial
#==================================================================================================================
#add_executable(DCSP_Tutorial_1 tutorial/DCSP_tutorial_multiinput.c)
#target_link_libraries(DCSP_Tutorial_1 PUBLIC ${NIDAQmx_LIB})
#target_include_directories(DCSP_Tutorial_1 PUBLIC Library)
#
#add_executable(DCSP_Tutorial_2 tutorial/DCSP_tutorial.c)
#target_link_libraries(DCSP_Tutorial_2 PUBLIC ${NIDAQmx_LIB})
#target_include_directories(DCSP_Tutorial_2 PUBLIC Library)
#
#add_executable(DCSP_Tutorial_3 tutorial/DSCP_aliasing.c)
#target_link_libraries(DCSP_Tutorial_3 PUBLIC ${NIDAQmx_LIB})
#target_include_directories(DCSP_Tutorial_3 PUBLIC Library)
#
#add_executable(example2 tutorial/example2.c)
#target_link_libraries(example2 PUBLIC ${NIDAQmx_LIB})
#target_include_directories(example2 PUBLIC Library)


#==================================================================================================================
# LAB
#==================================================================================================================

#===================
# STATIC MOTOR
#===================
# data collect
#add_executable(Static_Motor
#        LAB/static_motor/static_motor_modeling.c
#        ${SOURCE}
#        ${HEADER}
#)
#target_link_libraries(Static_Motor PUBLIC ${NIDAQmx_LIB})
#target_include_directories(Static_Motor PUBLIC
#        Library
#        include
#)
#
## linearlized data collect
#add_executable(Static_Motor_Linearization
#        LAB/static_motor/static_motor_linearization.c
#        ${SOURCE}
#        ${HEADER}
#)
#target_link_libraries(Static_Motor_Linearization PUBLIC ${NIDAQmx_LIB})
#target_include_directories(Static_Motor_Linearization PUBLIC
#        Library
#        include
#)



#===================
# GIMBAL
#===================

# dead_zone calculate
add_executable(Dead_zone_calculate
        LAB/static_motor/dead_zone_calculate.c
        ${SOURCE}
        ${HEADER}
)
target_link_libraries(Dead_zone_calculate PUBLIC ${NIDAQmx_LIB})
target_include_directories(Dead_zone_calculate PUBLIC
        Library
        include
)






# gimbal_modeling
add_executable(Static_Motor_Gimbal
        LAB/static_motor/static_motor_gimbal_modeling.c
        ${SOURCE}
        ${HEADER}
)
target_link_libraries(Static_Motor_Gimbal PUBLIC ${NIDAQmx_LIB})
target_include_directories(Static_Motor_Gimbal PUBLIC
        Library
        include
)


# gimbal linearlized data validation
add_executable(Static_Motor_Gimbal_Linearization_validation
        LAB/static_motor/static_motor_gimbal_linearized_validation.c
        ${SOURCE}
        ${HEADER}
)

target_link_libraries(Static_Motor_Gimbal_Linearization_validation PUBLIC ${NIDAQmx_LIB})
target_include_directories(Static_Motor_Gimbal_Linearization_validation PUBLIC
        Library
        include
)



# sin wave data collect
add_executable(Static_Motor_SineWave
        LAB/static_motor/sin_wave_cmd.c
        ${SOURCE}
        ${HEADER}
)
target_link_libraries(Static_Motor_SineWave PUBLIC ${NIDAQmx_LIB})
target_include_directories(Static_Motor_SineWave PUBLIC
        Library
        include
)


# potential meter data collect
add_executable(Potential_Meter_Radius_Calculate
        LAB/static_motor/potential_meter_value_read.c
        ${SOURCE}
        ${HEADER}
)
target_link_libraries(Potential_Meter_Radius_Calculate PUBLIC ${NIDAQmx_LIB})
target_include_directories(Potential_Meter_Radius_Calculate PUBLIC
        Library
        include
)


# potential meter data collect
add_executable(Controller_Validation
        LAB/static_motor/controller_validation.c
        ${SOURCE}
        ${HEADER}
)

target_link_libraries(Controller_Validation PUBLIC ${NIDAQmx_LIB})
target_include_directories(Controller_Validation PUBLIC
        Library
        include
)
