cmake_minimum_required(VERSION 2.8.12)

project (imgui)
set (LIBRARY_NAME ${PROJECT_NAME})
set(SOURCES     imgui.cpp
                imgui_demo.cpp
                imgui_draw.cpp
                imgui_impl_opengl3.cpp
                imgui_impl_win32.cpp
                imgui_widgets.cpp
)
find_package(OpenGL REQUIRED)
find_package(GLEW REQUIRED)

add_library(${LIBRARY_NAME} ${SOURCES})
target_include_directories(${LIBRARY_NAME} PRIVATE ${GLEW_INCLUDE_DIR} ${OPENGL_INCLUDE_DIR})
target_include_directories(${LIBRARY_NAME} PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>
                                                  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
                                                  $<INSTALL_INTERFACE:${INSTALL_INCLUDE_DIR}>)

install(
    TARGETS ${PROJECT_NAME} EXPORT dearImGuiInstall
)