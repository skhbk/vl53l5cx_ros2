project(uld)

set(VL53L5CX_SOURCES
    ${PROJECT_SOURCE_DIR}/src/vl53l5cx_api.c
    ${PROJECT_SOURCE_DIR}/platform/platform.c
    ${PROJECT_SOURCE_DIR}/src/vl53l5cx_plugin_xtalk.c)
set(VL53L5CX_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/inc
                          ${PROJECT_SOURCE_DIR}/platform)

add_library(uld STATIC ${VL53L5CX_SOURCES})
target_include_directories(uld PUBLIC ${VL53L5CX_INCLUDE_DIRS})
target_compile_definitions(uld PUBLIC VL53L5CX_ENABLE_DISTANCE_MM
                                      VL53L5CX_ENABLE_TARGET_STATUS)
