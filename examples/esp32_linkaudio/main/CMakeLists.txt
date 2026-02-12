set(CMAKE_CXX_STANDARD 17)

# The audio platform headers are in the same directory as main.cpp
# The common linkaudio headers are in ../../linkaudio
set(LINKAUDIO_COMMON_DIR ${CMAKE_CURRENT_LIST_DIR}/../../linkaudio)

idf_component_register(
    SRCS "main.cpp"
    INCLUDE_DIRS "." "${LINKAUDIO_COMMON_DIR}"
)

if(NOT DEFINED LINK_ESP_TASK_CORE_ID)
  set(LINK_ESP_TASK_CORE_ID 0)
endif()

# Apply definitions to the component library
target_compile_definitions(${COMPONENT_LIB} PRIVATE
    LINK_ESP_TASK_CORE_ID=0
    #${LINK_ESP_TASK_CORE_ID}
    LINK_AUDIO=1
    LINKHUT_AUDIO_PLATFORM_ESP32=1
)

# Use -fexceptions here
target_compile_options(${COMPONENT_LIB} PRIVATE -fexceptions)

# Link the Ableton library
include(../../../AbletonLinkConfig.cmake)
target_link_libraries(${COMPONENT_LIB} PUBLIC Ableton::Link)
