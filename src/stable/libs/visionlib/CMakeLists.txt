list_subdirectories( LIST_LIBS ${CMAKE_CURRENT_SOURCE_DIR} 1)

FOREACH (libs ${LIST_LIBS})
        ADD_SUBDIRECTORY (${libs})
ENDFOREACH()

# Generate documentation if doxygen was found
# If doxygen is installed we will generate documentation using "make doc"
if(DOXYGEN_FOUND)
    get_filename_component(DOC_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
    set(DOXYGEN_INPUT ${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in)
    set(DOXYGEN_FILE ${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile)
    set(DOXYGEN_OUTPUT ${DOXYGEN_OUTPUT_BASE}/libs/${DOC_NAME}/)
    file(MAKE_DIRECTORY ${DOXYGEN_OUTPUT})
    configure_file(${DOXYGEN_INPUT} ${DOXYGEN_FILE} @ONLY)
    add_custom_target(doc_${DOC_NAME} ${DOXYGEN_EXECUTABLE} ${DOXYGEN_FILE} | grep "Parsing file"
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
        COMMENT "Generating: library ${DOC_NAME}" VERBATIM
        DEPENDS ${DOXYGEN_INPUT}
    )
    add_dependencies(doc doc_${DOC_NAME})
endif(DOXYGEN_FOUND)


