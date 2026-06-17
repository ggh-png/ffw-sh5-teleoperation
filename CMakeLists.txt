cmake_minimum_required(VERSION 3.20)
project(FFW_SH5_Teleop CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ── External packages (apt) ──────────────────────────────────────────────────
find_package(OpenGL REQUIRED)
find_package(glfw3  REQUIRED)
find_package(GLEW   REQUIRED)
find_package(Bullet REQUIRED)

# tinyxml2: try cmake config first, fall back to manual
find_package(tinyxml2 CONFIG QUIET)
if(NOT tinyxml2_FOUND)
    find_library(TINYXML2_LIB tinyxml2 REQUIRED)
    find_path(TINYXML2_INC tinyxml2.h REQUIRED)
    add_library(tinyxml2_compat INTERFACE)
    target_link_libraries(tinyxml2_compat INTERFACE ${TINYXML2_LIB})
    target_include_directories(tinyxml2_compat INTERFACE ${TINYXML2_INC})
    add_library(tinyxml2::tinyxml2 ALIAS tinyxml2_compat)
endif()

# ── ImGui (vendored submodule) ───────────────────────────────────────────────
set(IMGUI_DIR ${CMAKE_SOURCE_DIR}/third_party/imgui)
set(IMGUI_SOURCES
    ${IMGUI_DIR}/imgui.cpp
    ${IMGUI_DIR}/imgui_draw.cpp
    ${IMGUI_DIR}/imgui_tables.cpp
    ${IMGUI_DIR}/imgui_widgets.cpp
    ${IMGUI_DIR}/backends/imgui_impl_glfw.cpp
    ${IMGUI_DIR}/backends/imgui_impl_opengl3.cpp
)

# ── Application sources ──────────────────────────────────────────────────────
file(GLOB_RECURSE APP_SOURCES CONFIGURE_DEPENDS src/*.cpp)

add_executable(ffw_teleop ${APP_SOURCES} ${IMGUI_SOURCES})

target_include_directories(ffw_teleop PRIVATE
    src
    ${IMGUI_DIR}
    ${IMGUI_DIR}/backends
    ${BULLET_INCLUDE_DIRS}
)

target_link_libraries(ffw_teleop PRIVATE
    OpenGL::GL
    glfw
    GLEW::GLEW
    ${BULLET_LIBRARIES}
    tinyxml2::tinyxml2
)

target_compile_options(ffw_teleop PRIVATE -O2 -Wall -Wextra -Wno-unused-parameter)

# ── Copy assets to build dir ─────────────────────────────────────────────────
add_custom_command(TARGET ffw_teleop POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        ${CMAKE_SOURCE_DIR}/assets $<TARGET_FILE_DIR:ffw_teleop>/assets
    COMMENT "Copying assets"
)
