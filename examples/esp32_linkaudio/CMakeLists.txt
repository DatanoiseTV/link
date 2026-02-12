cmake_minimum_required(VERSION 3.10)

set(EXTRA_COMPONENT_DIRS $ENV{IDF_PATH}/examples/common_components/protocol_examples_common)

#set(EXTRA_COMPONENT_DIRS $ENV{IDF_PATH}/examples/common_components/protocol_examples_common)



include($ENV{IDF_PATH}/tools/cmake/project.cmake)

project(link_esp32_linkaudio_example)
