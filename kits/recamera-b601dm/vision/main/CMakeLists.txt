# Device sources; tests/ builds the portable ones on the host.
set(srcs
    ${CMAKE_CURRENT_LIST_DIR}/main.cpp
    ${CMAKE_CURRENT_LIST_DIR}/vision_control.cpp
    ${CMAKE_CURRENT_LIST_DIR}/result_json.cpp
    ${CMAKE_CURRENT_LIST_DIR}/http_control.cpp
    ${CMAKE_CURRENT_LIST_DIR}/aruco_stage.cpp
    ${CMAKE_CURRENT_LIST_DIR}/marker_filter.cpp
    ${CMAKE_CURRENT_LIST_DIR}/camera_model.cpp
    ${CMAKE_CURRENT_LIST_DIR}/detector.cpp
)

component_register(
    COMPONENT_NAME main
    SRCS ${srcs}
    INCLUDE_DIRS ${CMAKE_CURRENT_LIST_DIR}
    PRIVATE_REQUIREDS app_framework apriltag sscma-micro sophgo debug_stream rtsp_server ha_mqtt onvif_service json ssl crypto cares
)
