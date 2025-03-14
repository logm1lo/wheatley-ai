file(GLOB_RECURSE UI_SRCS ./ui/*.c)

set(COMMON_SRC
	"../deps/livekit-protocol-generated/livekit_models.pb-c.c"
	"../deps/livekit-protocol-generated/livekit_rtc.pb-c.c"
	"webrtc.cpp"
	"websocket.cpp"
	"main.cpp")

if(IDF_TARGET STREQUAL linux)
	idf_component_register(
		SRCS ${COMMON_SRC}
		INCLUDE_DIRS "." "../deps/livekit-protocol-generated"
		REQUIRES peer esp-libopus esp_http_client protobuf-c)
else()
	idf_component_register(
		SRCS ${COMMON_SRC} "wifi.cpp" "media.cpp"  "cmd.cpp" ${UI_SRCS}
		REQUIRES driver protobuf-c esp_wifi nvs_flash esp_websocket_client peer esp_psram esp-libopus esp_http_client console
		INCLUDE_DIRS "." "./ui" "../deps/livekit-protocol-generated")
endif()

idf_component_get_property(lib peer COMPONENT_LIB)
target_compile_options(${lib} PRIVATE -Wno-error=restrict)
target_compile_options(${lib} PRIVATE -Wno-error=stringop-truncation)

idf_component_get_property(lib srtp COMPONENT_LIB)
target_compile_options(${lib} PRIVATE -Wno-error=incompatible-pointer-types)

idf_component_get_property(lib esp-libopus COMPONENT_LIB)
target_compile_options(${lib} PRIVATE -Wno-error=maybe-uninitialized)
target_compile_options(${lib} PRIVATE -Wno-error=stringop-overread)
