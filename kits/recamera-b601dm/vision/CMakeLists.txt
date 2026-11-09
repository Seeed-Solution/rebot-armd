cmake_minimum_required(VERSION 3.5.0)

# rebot-vision: built against an sscma-example-sg200x checkout used as an SDK.
#   SSCMA_ROOT        sscma-example-sg200x at the commit in sscma.lock (env or -D)
#   SG200X_SDK_PATH   reCamera OS SDK (env), toolchain riscv64-unknown-linux-musl-* in PATH
# scripts/build_vision.sh runs this inside the tpuc_dev container.

project(rebot-vision VERSION 1.2.1 LANGUAGES C CXX)

get_filename_component(PROJECT_DIR ${CMAKE_CURRENT_LIST_DIR} ABSOLUTE)

if(NOT SSCMA_ROOT)
    set(SSCMA_ROOT $ENV{SSCMA_ROOT})
endif()
if(NOT SSCMA_ROOT OR NOT EXISTS "${SSCMA_ROOT}/cmake/project.cmake")
    message(FATAL_ERROR "Set SSCMA_ROOT to an sscma-example-sg200x checkout (see sscma.lock)")
endif()
get_filename_component(ROOT_DIR "${SSCMA_ROOT}" ABSOLUTE)

option(REBOT_VISION_ALLOW_UNPINNED "Build against an SSCMA_ROOT that is not at the sscma.lock commit" OFF)
file(STRINGS "${CMAKE_CURRENT_LIST_DIR}/sscma.lock" _lock REGEX "^commit=")
string(REPLACE "commit=" "" SSCMA_LOCK_COMMIT "${_lock}")
execute_process(COMMAND git -c safe.directory=* -C "${ROOT_DIR}" rev-parse HEAD OUTPUT_VARIABLE SSCMA_HEAD
                OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_QUIET)
if(NOT SSCMA_HEAD STREQUAL SSCMA_LOCK_COMMIT)
    if(REBOT_VISION_ALLOW_UNPINNED)
        message(WARNING "SSCMA_ROOT is at '${SSCMA_HEAD}', sscma.lock pins ${SSCMA_LOCK_COMMIT}")
    else()
        message(FATAL_ERROR "SSCMA_ROOT is at '${SSCMA_HEAD}', sscma.lock pins ${SSCMA_LOCK_COMMIT} "
                            "(-DREBOT_VISION_ALLOW_UNPINNED=ON to override)")
    endif()
endif()
message(STATUS "SSCMA_ROOT: ${ROOT_DIR} @ ${SSCMA_HEAD}")

include(${ROOT_DIR}/cmake/toolchain-riscv64-linux-musl-x86_64.cmake)
include(${ROOT_DIR}/cmake/project.cmake)
