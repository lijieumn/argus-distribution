
find_package(glfw3 QUIET REQUIRED)
find_package(OpenGL QUIET REQUIRED)
find_package(GLEW QUIET REQUIRED)

file(GLOB DISPLAY_SOURCES "display/*")
add_library(display ${DISPLAY_SOURCES})
target_link_libraries(display ${OPENGL_LIBRARIES} glfw ${GLEW_LIBRARIES})

file(GLOB INTEGRATORS_SOURCES "integrators/*")
add_library(integrators ${INTEGRATORS_SOURCES})
target_link_libraries(integrators ${CMAKE_PROJECT_NAME}_bogus)

file(GLOB SIMULATORS_SOURCES "simulators/*")
add_library(simulators ${SIMULATORS_SOURCES})
target_link_libraries(simulators arcsim display integrators)

file(GLOB SYSTEM_SOURCES "applications/*")
add_library(applications ${SYSTEM_SOURCES})
target_link_libraries(applications simulators display)