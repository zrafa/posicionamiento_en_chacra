/*
 * lidar.cpp: funciones de acceso a los datos del lidar
 *
 */

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
#include <lidar.h>
#include <db.h>
#include <unistd.h>

// OpenCV
#include <opencv2/opencv.hpp>

using namespace std;

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 


// Leer los datos del archivo lidar.txt
vector<lidar_data> datos_lidar;


// leer los datos del archivo lidar.txt
vector<lidar_data> lidar_load(const string& filename) {
    vector<lidar_data> datos;
    ifstream archivo(filename);
    string linea;
    string campo1, campo2, campo3;

    while (getline(archivo, linea)) {
        stringstream ss(linea);
        string token;
        lidar_data data;

        // Parsear la línea
        ss >> campo1 >> data.marca_us >> data.marca_ms;

        // Extraer la distancia y el tiempo desde el primer campo
        stringstream ss_campo1(campo1);
        string aux;
        getline(ss_campo1, aux, ':');  // 000
        getline(ss_campo1, aux, ':');  // 00102 (distancia)
        data.distancia = stoi(aux);
        getline(ss_campo1, aux, ':');  // 000002 (tiempo de demora)
        data.tiempo_ms = stoi(aux);

            // Aplicar la condición de distancia y tiempo
            if (data.distancia < 200 && data.tiempo_ms > 10) {
                data.distancia = 400;
	    }
        datos.push_back(data);
    }
    return datos;
}




// Encontrar la distancia más cercana a una marca de tiempo
int lidar_get_distance(long long tiempo_us) {
    int distancia = -1;
    long long menorDiferencia = numeric_limits<long long>::max();

    for (const auto& dato : datos_lidar) {
        long long diferencia = abs(dato.marca_us - tiempo_us);

        if (diferencia < menorDiferencia) {
            menorDiferencia = diferencia;
                distancia = dato.distancia;
        }
    }

    return distancia;
}




// ------------------------------------------------------------------------------------------------


