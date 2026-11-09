cmake_minimum_required(VERSION 3.10)
project(rebot-vision-host-tests LANGUAGES C CXX)

# Host-only: no SSCMA SDK, CVI or camera. Builds the portable parts of rebot-vision
# (mode/trigger control, HTTP server, result JSON, marker filter, camera model).
#
#   cmake -S kits/recamera-b601dm/vision/tests -B target/vision-tests
#   cmake --build target/vision-tests && ctest --test-dir target/vision-tests
#
# nlohmann/json: SSCMA_ROOT/components/json when SSCMA_ROOT is set (env or -D), else the
# system package (Debian/Ubuntu: nlohmann-json3-dev).

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(Threads REQUIRED)

set(MAIN ${CMAKE_CURRENT_LIST_DIR}/../main)
if(NOT SSCMA_ROOT AND DEFINED ENV{SSCMA_ROOT})
    # cached, so a re-configure triggered by the build keeps it
    set(SSCMA_ROOT $ENV{SSCMA_ROOT} CACHE PATH "sscma-example-sg200x checkout")
endif()
if(SSCMA_ROOT AND EXISTS ${SSCMA_ROOT}/components/json/json.hpp)
    add_library(rv_json INTERFACE)
    target_include_directories(rv_json INTERFACE ${SSCMA_ROOT}/components/json)
else()
    find_package(nlohmann_json 3 REQUIRED)
    find_path(NLOHMANN_JSON_DIR nlohmann/json.hpp REQUIRED)
    add_library(rv_json INTERFACE)
    # the sources include "json.hpp"
    target_include_directories(rv_json INTERFACE ${NLOHMANN_JSON_DIR}/nlohmann)
endif()

enable_testing()

add_executable(vision_control_test
    ${CMAKE_CURRENT_LIST_DIR}/vision_control_test.cpp
    ${MAIN}/vision_control.cpp
    ${MAIN}/result_json.cpp
    ${MAIN}/http_control.cpp)
target_include_directories(vision_control_test PRIVATE ${MAIN})
target_compile_options(vision_control_test PRIVATE -Wall -Wextra)
target_link_libraries(vision_control_test PRIVATE rv_json Threads::Threads)
add_test(NAME vision_control COMMAND vision_control_test)

add_executable(camera_model_test ${CMAKE_CURRENT_LIST_DIR}/camera_model_test.cpp ${MAIN}/camera_model.cpp)
target_include_directories(camera_model_test PRIVATE ${MAIN})
target_compile_options(camera_model_test PRIVATE -Wall -Wextra)
add_test(NAME camera_model COMMAND camera_model_test)

add_executable(marker_filter_test ${CMAKE_CURRENT_LIST_DIR}/marker_filter_test.cpp ${MAIN}/marker_filter.cpp)
target_include_directories(marker_filter_test PRIVATE ${MAIN})
target_compile_options(marker_filter_test PRIVATE -Wall -Wextra)
add_test(NAME marker_filter COMMAND marker_filter_test)

add_executable(capture_time_test ${CMAKE_CURRENT_LIST_DIR}/capture_time_test.cpp)
target_include_directories(capture_time_test PRIVATE ${MAIN})
target_compile_options(capture_time_test PRIVATE -Wall -Wextra)
add_test(NAME capture_time COMMAND capture_time_test)

# ArUco stage benchmark (time per frame and recall, baseline vs tuned detector settings):
# needs the apriltag sources from SSCMA_ROOT/components/apriltag, so it is only built
# when SSCMA_ROOT is set. Built with -O2 so the times compare the settings, not -O0 code.
if(SSCMA_ROOT AND EXISTS ${SSCMA_ROOT}/components/apriltag/apriltag.h)
    set(AT ${SSCMA_ROOT}/components/apriltag)
    file(GLOB AT_SRCS ${AT}/*.c ${AT}/common/*.c ${AT}/aruco/*.c)
    add_library(rv_apriltag STATIC ${AT_SRCS})
    target_include_directories(rv_apriltag PUBLIC ${AT} ${AT}/common ${AT}/aruco)
    target_compile_options(rv_apriltag PRIVATE -O2 -w)
    target_link_libraries(rv_apriltag PUBLIC Threads::Threads m)

    add_executable(aruco_bench ${CMAKE_CURRENT_LIST_DIR}/aruco_bench.cpp ${MAIN}/aruco_stage.cpp
                               ${MAIN}/tag_pose.cpp ${MAIN}/marker_filter.cpp ${MAIN}/camera_model.cpp)
    target_include_directories(aruco_bench PRIVATE ${MAIN})
    target_compile_options(aruco_bench PRIVATE -O2 -Wall -Wextra)
    target_link_libraries(aruco_bench PRIVATE rv_apriltag rv_json)
    target_link_options(aruco_bench PRIVATE -Wl,--wrap=malloc,--wrap=calloc,--wrap=realloc)
    add_test(NAME aruco_bench COMMAND aruco_bench --scenes 200 --reps 1 --check)
    # quad_decimate default vs no decimation (recall >= 40 px, corner error)
    add_test(NAME aruco_decimate COMMAND aruco_bench --scenes 500 --reps 1 --sweep 1,2 --check)
endif()
