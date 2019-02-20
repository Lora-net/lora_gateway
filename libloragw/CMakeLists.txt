set(TARGET loragw)

add_library(${TARGET} "")

# -- add additional debug options
# Set the DEBUG_* to 1 to activate debug mode in individual modules.
# Warning: that makes the module *very verbose*, do not use for production
option(DEBUG_AUX "Active debug mode in AUX module" OFF)
option(DEBUG_SPI "Active debug mode in SPI module" OFF)
option(DEBUG_REG "Active debug mode in REG module" OFF)
option(DEBUG_HAL "Active debug mode in HAL module" OFF)
option(DEBUG_GPIO "Active debug mode in GPIO module" OFF)
option(DEBUG_LBT "Active debug mode in LBT module" OFF)
option(DEBUG_GPS "Active debug mode in GPS module" OFF)

message("-- Build with debug AUX: ${DEBUG_AUX}")
message("-- Build with debug SPI: ${DEBUG_SPI}")
message("-- Build with debug REG: ${DEBUG_REG}")
message("-- Build with debug HAL: ${DEBUG_HAL}")
message("-- Build with debug GPIO: ${DEBUG_GPIO}")
message("-- Build with debug LBT: ${DEBUG_LBT}")
message("-- Build with debug GPS: ${DEBUG_GPS}")

# --  add the compile options
target_compile_options(
	${TARGET}
	PRIVATE
	-Werror
	-Wall
	-Wextra
)

target_sources(${TARGET}
	PRIVATE
		${CMAKE_CURRENT_LIST_DIR}/src/loragw_aux.c
		${CMAKE_CURRENT_LIST_DIR}/src/loragw_fpga.c
		${CMAKE_CURRENT_LIST_DIR}/src/loragw_gps.c
		${CMAKE_CURRENT_LIST_DIR}/src/loragw_hal.c
		${CMAKE_CURRENT_LIST_DIR}/src/loragw_lbt.c
		${CMAKE_CURRENT_LIST_DIR}/src/loragw_radio.c
		${CMAKE_CURRENT_LIST_DIR}/src/loragw_reg.c
		${CMAKE_CURRENT_LIST_DIR}/src/loragw_spi.native.c
)

# --  add the public headers
set (${TARGET}_PUBLIC_HEADERS
	${CMAKE_CURRENT_LIST_DIR}/inc/loragw_aux.h
	${CMAKE_CURRENT_LIST_DIR}/inc/loragw_fpga.h
	${CMAKE_CURRENT_LIST_DIR}/inc/loragw_gps.h
	${CMAKE_CURRENT_LIST_DIR}/inc/loragw_hal.h
	${CMAKE_CURRENT_LIST_DIR}/inc/loragw_lbt.h
	${CMAKE_CURRENT_LIST_DIR}/inc/loragw_radio.h
	${CMAKE_CURRENT_LIST_DIR}/inc/loragw_reg.h
)

target_include_directories(${TARGET}
	PRIVATE
		${CMAKE_CURRENT_LIST_DIR}
		${CMAKE_CURRENT_LIST_DIR}/inc
	PUBLIC
		$<INSTALL_INTERFACE:include>
		$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/inc>
)

configure_file(${CMAKE_CURRENT_LIST_DIR}/${TARGET}_config.h.in "${CMAKE_CURRENT_BINARY_DIR}/config.h" @ONLY)

target_link_libraries(${TARGET}
	PUBLIC
		Threads::Threads
		m
)

set_target_properties(${TARGET} PROPERTIES VERSION ${lora_gateway_VERSION_SHORT})
set_target_properties(${TARGET} PROPERTIES SOVERSION ${lora_gateway_VERSION_SHORT})
set_target_properties(${TARGET} PROPERTIES PUBLIC_HEADER "${CMAKE_CURRENT_BINARY_DIR}/config.h;${${TARGET}_PUBLIC_HEADERS}")

# --  add the install targets
install (TARGETS ${TARGET}
	EXPORT ${TARGET}_targets
  	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT shlib
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
	PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${TARGET}
	INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${TARGET}
)

# -- add pkg config file
configure_file ("${CMAKE_CURRENT_LIST_DIR}/${TARGET}.pc.in" "${PROJECT_BINARY_DIR}/${TARGET}.pc" @ONLY)
install (FILES ${PROJECT_BINARY_DIR}/${TARGET}.pc DESTINATION lib/pkgconfig)

# -- write cmake package config file
write_basic_package_version_file(
	"${CMAKE_CURRENT_BINARY_DIR}/${TARGET}/${TARGET}-config-version.cmake"
	VERSION ${lora_gateway_VERSION}
	COMPATIBILITY AnyNewerVersion
)

export(EXPORT ${TARGET}_targets
	FILE "${CMAKE_CURRENT_BINARY_DIR}/${TARGET}/${TARGET}-targets.cmake"
	NAMESPACE Semtech::
)

configure_file(${PROJECT_SOURCE_DIR}/cmake/${TARGET}-config.cmake
	"${CMAKE_CURRENT_BINARY_DIR}/${TARGET}/${TARGET}-config.cmake"
	COPYONLY
)

set(ConfigPackageLocation lib/cmake/${TARGET})

install(EXPORT ${TARGET}_targets
	FILE ${TARGET}-targets.cmake
	NAMESPACE Semtech::
	DESTINATION ${ConfigPackageLocation}
)

install(
	FILES ${PROJECT_SOURCE_DIR}/cmake/${TARGET}-config.cmake "${CMAKE_CURRENT_BINARY_DIR}/${TARGET}/${TARGET}-config-version.cmake"
	DESTINATION ${ConfigPackageLocation}
	COMPONENT Devel
)

# -- add test programs
foreach(TEST test_loragw_spi test_loragw_gps test_loragw_reg test_loragw_hal test_loragw_cal)
	add_executable(${TEST} "")

	target_sources(${TEST}
		PRIVATE
			${CMAKE_CURRENT_LIST_DIR}/tst/${TEST}.c
	)

	target_include_directories(${TEST}
		PRIVATE
			$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
			$<INSTALL_INTERFACE:include>
			${CMAKE_CURRENT_LIST_DIR}/inc
			${CMAKE_CURRENT_BINARY_DIR}
	)

	target_link_libraries(${TEST}
		PRIVATE
			loragw
	)

	install (
		TARGETS ${TEST}
		RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
	)

endforeach()

