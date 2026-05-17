cmake_minimum_required(VERSION 3.10)

project(rasteriser)

add_executable(rasteriser
    rasteriser.cpp
    Vector2.hpp
    Vector3.hpp
    Mesh.hpp
    LinAlg.hpp
    Image.hpp
    Light.hpp
    3rdParty/lodepng/lodepng.cpp
)

target_include_directories(rasteriser PRIVATE
    ${CMAKE_SOURCE_DIR}/3rdParty/lodepng
    ${CMAKE_SOURCE_DIR}/3rdParty/eigen-3.4.1
)