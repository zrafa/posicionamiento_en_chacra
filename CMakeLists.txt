
cmake_minimum_required(VERSION 3.10)

# Nombre del proyecto
project(MiProyecto)

# Indicar que se va a usar C++ 11 o superior
set(CMAKE_CXX_STANDARD 11)

# Buscar OpenCV
find_package(OpenCV REQUIRED)

# Incluir directorios de OpenCV
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PROJECT_NAME} PUBLIC include/)


# Agregar el ejecutable
add_executable(mi_proyecto src/detectar_troncos.cpp)

# Enlazar la librería de OpenCV
target_link_libraries(mi_proyecto ${OpenCV_LIBS})

