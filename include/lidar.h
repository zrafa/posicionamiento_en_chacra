/*
 * lidar.h funciones de acceso a los datos del lidar
 */

#ifndef LIDAR_H
#define LIDAR_H

#include <iostream>
#include <vector>
#include <fstream>
#include <chrono>

#include <sstream>
#include <cmath>
#include <limits>
#include <algorithm>

#include <vars.h>
#include <gps.h>
#include <ui_gfx.h>
#include <db.h>
#include <unistd.h>

// OpenCV
#include <opencv2/opencv.hpp>

using namespace std;

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 




// Estructura para almacenar la información del lidar
struct lidar_data {
    int distancia;
    int tiempo_ms;
    long long marca_ms;
    long long marca_us;
};

// Leer los datos del archivo lidar.txt
extern vector<lidar_data> datos_lidar;


// leer los datos del archivo lidar.txt
vector<lidar_data> lidar_load(const string& filename);

// Encontrar la distancia más cercana a una marca de tiempo
int lidar_get_distance(long long tiempo_us);

#endif // LIDAR_H



