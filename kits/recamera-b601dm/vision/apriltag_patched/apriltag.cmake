# apriltag for rebot-vision: the sources of sscma-example-sg200x components/apriltag (at
# the sscma.lock commit) with the .c files of this directory replacing the ones of the same
# name. They are BSD-2-Clause (THIRD_PARTY_NOTICES.md); each patched file lists its changes
# under its license header. Used by main/CMakeLists.txt and tests/CMakeLists.txt.
#   in:  RV_APRILTAG_SDK           the SDK's components/apriltag directory
#   out: RV_APRILTAG_SRCS          sources to build
#        RV_APRILTAG_INCLUDE_DIRS  this directory first, then the SDK's
set(_rv_at_patched_dir ${CMAKE_CURRENT_LIST_DIR})
file(GLOB RV_APRILTAG_SRCS ${RV_APRILTAG_SDK}/*.c ${RV_APRILTAG_SDK}/common/*.c ${RV_APRILTAG_SDK}/aruco/*.c)
file(GLOB _rv_at_patched ${_rv_at_patched_dir}/*.c)
foreach(_f IN LISTS _rv_at_patched)
    get_filename_component(_n ${_f} NAME)
    list(REMOVE_ITEM RV_APRILTAG_SRCS ${RV_APRILTAG_SDK}/${_n})
endforeach()
list(APPEND RV_APRILTAG_SRCS ${_rv_at_patched})
set(RV_APRILTAG_INCLUDE_DIRS ${_rv_at_patched_dir} ${RV_APRILTAG_SDK} ${RV_APRILTAG_SDK}/common ${RV_APRILTAG_SDK}/aruco)
