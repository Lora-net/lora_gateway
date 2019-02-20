
add_executable(util_spectral_scan "")
target_sources(util_spectral_scan
        PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/src/util_spectral_scan.c
)

target_link_libraries(util_spectral_scan
	PUBLIC
		loragw
)

set_target_properties(util_spectral_scan PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)

# add the install targets
install (
        TARGETS util_spectral_scan
  	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT shlib
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
	RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
