cmake_minimum_required (VERSION 2.6)
project (CG_UFFS)

find_package(OpenGL REQUIRED)

# Compile external dependencies 
add_subdirectory (external)

add_definitions(
	-DTW_STATIC
	-DTW_NO_LIB_PRAGMA
	-DTW_NO_DIRECT3D
	-DGLEW_STATIC
	-D_CRT_SECURE_NO_WARNINGS
)

add_executable(CG_UFFS
    sources/controls.cpp
    sources/main.cpp
    sources/objloader.cpp    
    sources/quaternion_utils.cpp
    sources/shader.cpp
    sources/tangentspace.cpp
    sources/text2D.cpp
    sources/texture.cpp
    sources/vboindexer.cpp
    sources/glerror.cpp

    include/controls.hpp    
    include/objloader.hpp
    include/quaternion_utils.hpp
    include/shader.hpp
    include/tangentspace.hpp
    include/text2D.hpp
    include/texture.hpp
    include/vboindexer.hpp
    include/glerror.hpp
)

include_directories(
    external/AntTweakBar-1.16/include/
    external/glfw-3.0.3/include/GLFW
    external/glm-0.9.4.0/
    external/glew-1.9.0/include/
	external/assimp-3.0.1270/include/
	include/
)

set(ALL_LIBS
    ${OPENGL_LIBRARY}
    GLFW_303
    GLEW_190
    ANTTWEAKBAR_116_OGLCORE_GLFW
)

target_link_libraries(CG_UFFS
    ${ALL_LIBS}
	assimp
)