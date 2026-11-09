# apriltag: the SDK component's sources with the kit's patched files (apriltag_patched/),
# registered as rv_apriltag instead of the SDK's apriltag component.
set(RV_APRILTAG_SDK ${ROOT_DIR}/components/apriltag)
include(${CMAKE_CURRENT_LIST_DIR}/../apriltag_patched/apriltag.cmake)
component_register(
    COMPONENT_NAME rv_apriltag
    SRCS ${RV_APRILTAG_SRCS}
    INCLUDE_DIRS ${RV_APRILTAG_INCLUDE_DIRS}
)

# Device sources; tests/ builds the portable ones on the host.
set(srcs
    ${CMAKE_CURRENT_LIST_DIR}/main.cpp
    ${CMAKE_CURRENT_LIST_DIR}/vision_control.cpp
    ${CMAKE_CURRENT_LIST_DIR}/result_json.cpp
    ${CMAKE_CURRENT_LIST_DIR}/http_control.cpp
    ${CMAKE_CURRENT_LIST_DIR}/aruco_stage.cpp
    ${CMAKE_CURRENT_LIST_DIR}/tag_pose.cpp
    ${CMAKE_CURRENT_LIST_DIR}/marker_filter.cpp
    ${CMAKE_CURRENT_LIST_DIR}/camera_model.cpp
    ${CMAKE_CURRENT_LIST_DIR}/detector.cpp
)

component_register(
    COMPONENT_NAME main
    SRCS ${srcs}
    INCLUDE_DIRS ${CMAKE_CURRENT_LIST_DIR}
    PRIVATE_REQUIREDS app_framework rv_apriltag sscma-micro sophgo debug_stream rtsp_server ha_mqtt onvif_service json ssl crypto cares
)
