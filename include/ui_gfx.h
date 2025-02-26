/*
 * ui_gfx.h: interfaz de usuario grafica
 *
 * Muestra una ventana completa con informacion en tiempo real
 */

#ifndef UI_GFX_H
#define UI_GFX_H

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

// OpenCV
#include <opencv2/opencv.hpp>

using namespace std;

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

#define rojo 0
#define verde 1


void mostrar_orientacion(double grados);
// Función para convertir coordenadas GPS a píxeles
cv::Point2f gps_to_pixel(double latitude, double longitude, 
		         double ref_lat, double ref_lon);

// Función para mostrar el GPS en la ventana
void mostrar_gps(cv::Mat &ventana_completa);
void mostrar_texto(cv::Mat &ventana_completa, ostringstream &texto, int x, int y);
void mostrar_distancia(cv::Mat &ventana_completa);

void mostrar_hileras_con_tractor(cv::Mat &ventana_completa, int nro_hilera, int nro_peral); 

// Función para mostrar imágenes en las posiciones deseadas
void mostrar_foto(const cv::Mat& foto_orig, int posicion);
void mostrar_ventana_completa(void);

#endif // UI_GFX_H
