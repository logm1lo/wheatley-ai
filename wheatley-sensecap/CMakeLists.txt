# For more information about build system see
# https://docs.espressif.com/projects/esp-idf/en/latest/api-guides/build-system.html
# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)

# Source
set(COMPONENTS src)
set(EXTRA_COMPONENT_DIRS "src" "components/srtp" "components/peer" "components/esp-libopus" "components/esp-protocols/components/esp_websocket_client")


if(IDF_TARGET STREQUAL linux)
	add_compile_definitions(LINUX_BUILD=1)
  list(APPEND EXTRA_COMPONENT_DIRS
    $ENV{IDF_PATH}/examples/protocols/linux_stubs/esp_stubs
    "components/esp-protocols/common_components/linux_compat/esp_timer"
    "components/esp-protocols/common_components/linux_compat/freertos"
    )
endif()

if(NOT DEFINED ENV{SKIP_PROTOBUF_GENERATE})
  execute_process(
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
    COMMAND "protoc" "--c_out=deps/livekit-protocol-generated" "-I=./deps/livekit-protocol" "livekit_rtc.proto" "livekit_models.proto"
    COMMAND_ERROR_IS_FATAL ANY
  )
endif()

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
add_compile_options(-fdiagnostics-color=always -Wno-error=format= -Wno-format)
project(wheatley-sensecap)
