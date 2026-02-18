
cmake_minimum_required(VERSION 3.1)
project(posicionamiento)


# EN EMBEBIDO ESTA LINEA
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native -mfpu=neon -ffast-math -march=armv7-a -mfpu=neon -mfloat-abi=hard -funroll-loops -O3")

# EN PC ESTAS LINEAS
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0")

# set(CMAKE_BUILD_TYPE Debug)


# EN EMBEBIDO ESTAS LINEAS
# find_package(PkgConfig REQUIRED)
# pkg_check_modules(OPENCV REQUIRED opencv)
# include_directories(${OPENCV_INCLUDE_DIRS})
# include_directories(${PROJECT_NAME} PUBLIC include/)
# link_directories(${OPENCV_LIBRARY_DIRS})


# EN PC ESTAS LINEAS
# Buscar OpenCV
find_package(OpenCV REQUIRED)
# Incluir directorios de OpenCV
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PROJECT_NAME} PUBLIC include/)


set(SOURCES
    src/gps.cpp
    src/ui_gfx.cpp
    src/detectar_troncos.cpp
    src/db.cpp
    src/lidar.cpp
    src/main.cpp
)



# EN EMBEBIDO ESTAS LINEAS
## Crear el ejecutable
#add_executable(posicionamiento ${SOURCES})
#target_link_libraries(posicionamiento ${OPENCV_LIBRARIES} pthread)


# EN PC ESTAS LINEAS
# Crear el ejecutable
add_executable(posicionamiento ${SOURCES})
add_executable(ground_truth src/ground.cpp)
# Enlazar la librería de OpenCV
target_link_libraries(posicionamiento ${OpenCV_LIBS})
target_link_libraries(ground_truth ${OpenCV_LIBS})





#cmake_minimum_required(VERSION 3.7)

# Nombre del proyecto
#project(posicionamiento)

# Indicar que se va a usar C++ 11 o superior
#set(CMAKE_CXX_STANDARD 11)

#find_package(PkgConfig REQUIRED)
#pkg_check_modules(OPENCV REQUIRED opencv)

#include_directories(${OPENCV_INCLUDE_DIRS})
#link_directories(${OPENCV_LIBRARY_DIRS})

## Buscar OpenCV
## find_package(OpenCV REQUIRED)

## Incluir directorios de OpenCV
## include_directories(${OpenCV_INCLUDE_DIRS})
#include_directories(${PROJECT_NAME} PUBLIC include/)

#set(SOURCES
#    src/gps.cpp
#    src/ui_gfx.cpp
#    src/detectar_troncos.cpp
#    src/db.cpp
#    src/lidar.cpp
#    src/main.cpp
#)
#
## Crear el ejecutable
#add_executable(posicionamiento ${SOURCES})
#add_executable(ground_truth src/ground.cpp)
#
## Enlazar la librería de OpenCV
#target_link_libraries(posicionamiento ${OpenCV_LIBS})
#target_link_libraries(ground_truth ${OpenCV_LIBS})

