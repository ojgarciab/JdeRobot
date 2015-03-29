SET( SOURCE_FILES  opencvdemo.cpp viewer.cpp)


include_directories(
    ${INTERFACES_CPP_DIR}
    ${LIBS_DIR}/
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${gtkmm_INCLUDE_DIRS}
    ${libglademm_INCLUDE_DIRS}
)


add_executable (opencvdemo ${SOURCE_FILES})


TARGET_LINK_LIBRARIES(opencvdemo
    ${CMAKE_THREAD_LIBS_INIT} 
    ${OpenCV_LIBRARIES}
    ${gtkmm_LIBRARIES}
    ${libglademm_LIBRARIES}
    colorspacesmm
    JderobotInterfaces
    ${ZeroCIce_LIBRARIES}
)

# Find and check dependencies
SET( DEPS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../../../Deps) # Directorio donde se encuentran las dependencias
include(${DEPS_DIR}/ice/CMakeLists.txt)
include(${DEPS_DIR}/gtk2/CMakeLists.txt)
include(${DEPS_DIR}/opencv/CMakeLists.txt)
include(${DEPS_DIR}/doxygen/CMakeLists.txt)


# If doxygen is installed we will generate documentation using "make doc"
if(DOXYGEN_FOUND)
    set(DOXYGEN_INPUT ${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in)
    set(DOXYGEN_FILE ${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile)
    set(DOXYGEN_OUTPUT ${CMAKE_CURRENT_SOURCE_DIR}/../../../../doc/components/opencvdemo/)
    file(MAKE_DIRECTORY ${DOXYGEN_OUTPUT})
    configure_file(${DOXYGEN_INPUT} ${DOXYGEN_FILE} @ONLY)
    add_custom_target(doc ${DOXYGEN_EXECUTABLE} ${DOXYGEN_FILE} | grep "Parsing file"
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
        COMMENT "Generating: component opencvdemo" VERBATIM
    	DEPENDS ${DOXYGEN_INPUT}
    )
endif(DOXYGEN_FOUND)

