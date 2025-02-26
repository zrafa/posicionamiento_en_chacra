/*
 * db.h : crea, carga y busca en hilera.db
 */


#ifndef DB_H // Guardas de inclusión para evitar múltiples inclusiones
#define DB_H

#include <iostream>
#include <vector>
#include <fstream>
#include <chrono>

#include <sstream>
#include <cmath>
#include <limits>
#include <algorithm>

#include <vars.h>
#include <db.h>
#include <unistd.h>

#include <opencv2/opencv.hpp>

using namespace std;

// ------------------------------ DB

struct frutal {
        int nro_arbol;
        int distancia;
        int diametro;
        int x1;         // lateral izquierdo del arbol en la foto
        int x2;         // lateral derecho del arbol en la foto
        double latitud;
        double longitud;
        cv::Mat image;  // falta foto
        // falta marca de tiempo
};

struct arbol_db {
    int id;
    int diametro_en_px;
    double diametro_en_cm;
    double latitud;
    double longitud;
    string foto;
    vector<cv::Mat> descriptores;
};


string db_get_foto(int n);
int db_buscar_por_diametro(double diametro_cm, int arbol_id);
void db_buscar_por_gps(int arbol_id, double latitud, double longitud, 
		       int *cual, double *distancia);
int db_buscar(const cv::Mat& fotoNueva);
void db_add(int id, int diametro_en_px, double diametro_en_cm, 
	    double latitud, double longitud, string foto);
void db_save(const string& archivo);
void db_load(const string& archivo);

// ---------------------------- fin de DB


#endif // DB_H
       
