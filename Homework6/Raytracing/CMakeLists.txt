cmake_minimum_required(VERSION 3.17)
project(Raytracing)

set(CMAKE_CXX_STANDARD 14)

include_directories(/usr/local/include)
add_executable(RayTracing main.cpp Object.hpp Vector.cpp Vector.hpp Sphere.hpp global.hpp Triangle.hpp Scene.cpp
        Scene.hpp Light.hpp AreaLight.hpp BVH.cpp BVH.hpp Bounds3.hpp Ray.hpp Material.hpp Intersection.hpp
        Renderer.cpp Renderer.hpp)