
add_executable(util_spi_stress "")
target_sources(util_spi_stress
        PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/src/util_spi_stress.c
)

target_link_libraries(util_spi_stress
	PUBLIC
		loragw
)

set_target_properties(util_spi_stress PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)

# add the install targets
install (
        TARGETS util_spi_stress
  	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT shlib
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
	RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
