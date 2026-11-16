cmake_minimum_required(VERSION 3.5)
project(m3_widget_gimbal CXX C)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=gnu99")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
add_definitions(-D_GNU_SOURCE -DSYSTEM_ARCH_LINUX)

if(NOT PSDK_ROOT)
    set(PSDK_ROOT /home/dji/test_nat/Payload-SDK-3.18.0)
endif()
set(PSDK_PLATFORM ${PSDK_ROOT}/samples/sample_c/platform/linux)
message(STATUS "PSDK root: ${PSDK_ROOT}")

include_directories(3rd_party/spdlog-1.8.2/include)
include_directories(${PSDK_ROOT}/psdk_lib/include)
include_directories(${PSDK_PLATFORM}/common)
include_directories(${PSDK_PLATFORM}/manifold3)
include_directories(common)
include_directories(module)

link_directories(${PSDK_ROOT}/psdk_lib/lib/aarch64-linux-gnu-gcc)

file(GLOB_RECURSE COM_SRC common/*.c*)
file(GLOB_RECURSE MODULES_CONF_SRC module/conf/*.c*)
file(GLOB_RECURSE MODULES_VEHICLE_SRC module/vehicle/*.c*)
file(GLOB_RECURSE MODULES_CONTROLLER_SRC module/controller/*.c*)
file(GLOB_RECURSE MODULES_WIDGET_SRC module/widget/*.c*)
file(GLOB_RECURSE MODULES_PIPELINE_SRC module/pipeline/*.c*)
file(GLOB_RECURSE MODULES_MOP_SRC module/mop/*.c*)
file(GLOB PSDK_OSAL_SRC ${PSDK_PLATFORM}/common/osal/*.c)
file(GLOB PSDK_HAL_SRC ${PSDK_PLATFORM}/manifold3/hal/*.c)

set(COM_LIBS pthread m dl)
set(DJI_PSDK_LIBS libpayloadsdk.a)

set(APP_NAME m3_widget_gimbal)
add_executable(${APP_NAME} app/m3_widget_gimbal.cc
    ${COM_SRC} ${MODULES_CONF_SRC} ${MODULES_VEHICLE_SRC}
    ${MODULES_CONTROLLER_SRC} ${MODULES_WIDGET_SRC} ${MODULES_PIPELINE_SRC}
    ${MODULES_MOP_SRC}
    ${PSDK_OSAL_SRC} ${PSDK_HAL_SRC})

target_link_libraries(${APP_NAME} ${DJI_PSDK_LIBS} ${COM_LIBS})
