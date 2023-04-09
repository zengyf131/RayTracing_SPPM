CMAKE_MINIMUM_REQUIRED(VERSION 2.8)
PROJECT(FINAL CXX)

IF(NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE Release)
ENDIF()

ADD_SUBDIRECTORY(deps/vecmath)
find_package(OpenMP REQUIRED)

SET(FINAL_SOURCES
        src/image.cpp
        src/main.cpp
        src/scene_parser.cpp)

SET(FINAL_INCLUDES
        include/camera.hpp
        include/group.hpp
        include/hit.hpp
        include/image.hpp
        include/material.hpp
        include/object3d.hpp
        include/ray.hpp
        include/scene_parser.hpp
        include/transform.hpp
        include/SPPM.hpp
        include/objects/box.hpp
        include/objects/curve.hpp
        include/objects/light.hpp
        include/objects/mesh.hpp
        include/objects/plane.hpp
        include/objects/revsurface.hpp
        include/objects/sphere.hpp
        include/objects/triangle.hpp
        )

SET(CMAKE_CXX_STANDARD 11)
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)

ADD_EXECUTABLE(${PROJECT_NAME} ${FINAL_SOURCES} ${FINAL_INCLUDES})
TARGET_LINK_LIBRARIES(${PROJECT_NAME} vecmath OpenMP::OpenMP_CXX)
TARGET_INCLUDE_DIRECTORIES(${PROJECT_NAME} PRIVATE include)
