/*
 * gps.cpp: procesa las tramas del gps
 * dentro de las hileras de una chacra.
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
#include <db.h>
#include <unistd.h>

using namespace std;

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

// obtener latitud y longitud en base a una marca de tiempo
void gps_get_lat_lon(long long tiempo_us, double *latitud, double *longitud) 
{
    ifstream file("gps.txt");
    if (!file.is_open()) {
        cerr << "Error: No se pudo abrir el archivo gps.txt" << endl;
        return;
    }

    // Variables para almacenar la trama más cercana
    GPS_data closest_data;
    long long min_time_diff = numeric_limits<long long>::max();
    string line;
    long long prev_timestamp_us = 0;

    // Leer el archivo línea por línea
    while (getline(file, line)) {
        if (line.find("$GNRMC") != string::npos) {
            // Procesar la línea $GNRMC
            stringstream ss(line);
            string token;
            vector<string> tokens;
            while (getline(ss, token, ',')) {
                tokens.push_back(token);
            }

            // Extraer latitud y longitud
            double latitude = stod(tokens[3].substr(0, 2)) + stod(tokens[3].substr(2)) / 60.0;
            double longitude = stod(tokens[5].substr(0, 3)) + stod(tokens[5].substr(3)) / 60.0;
            if (tokens[4] == "S") latitude *= -1;
            if (tokens[6] == "W") longitude *= -1;

            // Calcular la diferencia de tiempo con el tiempo_us proporcionado
            long long time_diff = abs(prev_timestamp_us - tiempo_us);

            // Si esta trama es la más cercana hasta ahora, guardarla
            if (time_diff < min_time_diff) {
                min_time_diff = time_diff;
                closest_data = {prev_timestamp_us, latitude, longitude};
            }
        } else {
            // Extraer la marca de tiempo (en us)
            stringstream ss(line);
            ss >> prev_timestamp_us;
        }
    }

    file.close();

    // Verificar si min_time_diff está dentro del rango
    if (min_time_diff <= 1000000) {
	    // rango_inferior && min_time_diff <= rango_superior) {
        *latitud = (double) closest_data.latitude;
        *longitud = (double) closest_data.longitude;
    } else {
        *latitud = (double) -1.0;
        *longitud = (double) -1.0;
    }

}

