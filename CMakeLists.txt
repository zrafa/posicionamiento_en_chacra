
cmake_minimum_required(VERSION 3.10)

# Nombre del proyecto
project(posicionamiento)

# Indicar que se va a usar C++ 11 o superior
set(CMAKE_CXX_STANDARD 11)

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
)

# Crear el ejecutable
add_executable(posicionamiento ${SOURCES})
add_executable(ground_truth src/ground.cpp)

# Enlazar la librería de OpenCV
target_link_libraries(posicionamiento ${OpenCV_LIBS})
target_link_libraries(ground_truth ${OpenCV_LIBS})

