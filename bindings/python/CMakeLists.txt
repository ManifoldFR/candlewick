include(${PROJECT_SOURCE_DIR}/cmake/python.cmake)
include(${PROJECT_SOURCE_DIR}/cmake/stubs.cmake)

find_package(eigenpy 3.2.0 REQUIRED)

set(
  pycandlewick_SOURCES
  src/expose-debug.cpp
  src/expose-mesh-data.cpp
  src/expose-renderer.cpp
  src/module.cpp
)

set(PYLIB_INSTALL_DIR ${PYTHON_SITELIB}/${PROJECT_NAME})

Python3_add_library(pycandlewick MODULE WITH_SOABI ${pycandlewick_SOURCES})
target_link_libraries(pycandlewick PRIVATE candlewick eigenpy::eigenpy)
target_include_directories(
  pycandlewick
  PRIVATE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)
if(BUILD_PINOCCHIO_VISUALIZER)
  target_sources(pycandlewick PRIVATE src/expose-visualizer.cpp)
  # Add definition for the core library
  target_compile_definitions(
    pycandlewick
    PRIVATE CANDLEWICK_PYTHON_PINOCCHIO_SUPPORT
  )
endif()
if(BUILD_WITH_FFMPEG_SUPPORT)
  target_sources(pycandlewick PRIVATE src/expose-recording.cpp)
endif()

set_target_properties(
  pycandlewick
  PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${PROJECT_NAME}
)
if(UNIX)
  GET_RELATIVE_RPATH(${PYLIB_INSTALL_DIR} PYLIB_INSTALL_RPATH)
  set_target_properties(
    pycandlewick
    PROPERTIES INSTALL_RPATH "${PYLIB_INSTALL_RPATH}"
  )
endif()

install(
  TARGETS pycandlewick
  EXPORT ${TARGETS_EXPORT_NAME}
  LIBRARY DESTINATION ${PYLIB_INSTALL_DIR}
)

if(GENERATE_PYTHON_STUBS)
  LOAD_STUBGEN()
  GENERATE_STUBS(
    ${CMAKE_CURRENT_BINARY_DIR}
    ${PROJECT_NAME}.pycandlewick
    ${PYTHON_SITELIB}/candlewick
    pycandlewick
  )
endif(GENERATE_PYTHON_STUBS)

set(PYTHON_FILES __init__.py async_visualizer.py video_context.py)

foreach(pyfile ${PYTHON_FILES})
  PYTHON_INSTALL_ON_SITE(${PROJECT_NAME} ${pyfile})
endforeach()
