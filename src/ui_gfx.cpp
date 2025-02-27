/*
 * ui_gfx.cpp: interfaz de usuario grafica
 *
 * Muestra una ventana completa con informacion en tiempo real
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
#include <db.h>
#include <unistd.h>

// OpenCV
#include <opencv2/opencv.hpp>

using namespace std;

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

extern int distancia;
extern long long tiempo_us;
extern int tractor_en_peral;

cv::Mat ventana_completa;

int tractor_color = rojo;



using namespace cv;

void mostrar_orientacion(double grado) {
	int x = 1000;
	int y = 720;
    // Crear una imagen en blanco (ventana completa)
    // Mat imagen = Mat::zeros(600, 600, CV_8UC3);

    // Centro de la brújula
    //Point centro(ventana_completa.cols / 2, ventana_completa.rows / 2);
    Point centro(x, y);

    // Radio de la brújula
    int radio = 80;

    // limpiamos ese sector de la ventana con negro
    cv::Point punto1(x-radio, y-radio);   // (x1, y1)
    cv::Point punto2(x+radio, y+radio);  // (x2, y2)
    cv::rectangle(ventana_completa, punto1, punto2, cv::Scalar(0, 0, 0), -1); // -1 relleno 
									      //
    // Dibujar el círculo de la brújula
    circle(ventana_completa, centro, radio, Scalar(255, 255, 255), 2);

    // Dibujar las etiquetas N, E, S, W
    int fontFace = FONT_HERSHEY_SIMPLEX;
    double fontScale = 1.0;
    int thickness = 2;

    // Norte (N)
    putText(ventana_completa, "N", Point(centro.x - 10, centro.y - radio + 40), fontFace, fontScale, Scalar(0, 0, 255), thickness);

    // Este (E)
    putText(ventana_completa, "E", Point(centro.x + radio - 30, centro.y + 10), fontFace, fontScale, Scalar(0, 0, 255), thickness);

    // Sur (S)
    putText(ventana_completa, "S", Point(centro.x - 10, centro.y + radio - 20), fontFace, fontScale, Scalar(0, 0, 255), thickness);

    // Oeste (W)
    putText(ventana_completa, "W", Point(centro.x - radio + 10, centro.y + 10), fontFace, fontScale, Scalar(0, 0, 255), thickness);

    // Convertir el ángulo a radianes
    double radianes = grado * M_PI / 180.0;

    // Calcular el punto final de la flecha
    Point flecha_final(
        centro.x + static_cast<int>(radio * 0.8 * sin(radianes)),
        centro.y - static_cast<int>(radio * 0.8 * cos(radianes))
    );

    // Dibujar la flecha
    arrowedLine(ventana_completa, centro, flecha_final, Scalar(0, 255, 0), 3);

    // Mostrar la imagen en una ventana
//    imshow("Brújula", ventana_completa);
 //   waitKey(1);  // Actualizar la ventana
}

// Función para convertir coordenadas GPS a píxeles
cv::Point2f gps_to_pixel(double latitude, double longitude, 
		         double ref_lat, double ref_lon) 
{
    // Aproximación: 1 grado de latitud ≈ 111,320 metros, 1 grado de longitud ≈ 96,486 metros
    double lat_to_meters = 111320.0;
    double lon_to_meters = 96486.0;

    // Calcular la diferencia en metros respecto a una referencia
    double delta_lat = (latitude - ref_lat) * lat_to_meters;
    double delta_lon = (longitude - ref_lon) * lon_to_meters;

     // Convertir metros a píxeles (10 cm = 1 píxel)
    double escala = 10.0; // 1 metro = 10 píxeles (ya que 1 píxel = 10 cm)
    return cv::Point2f(delta_lon * escala, -delta_lat * escala); // Negativo para ajustar el eje Y
}


// Función para mostrar el GPS en la ventana
void mostrar_gps(cv::Mat &ventana_completa) 
{
    // Referencia de latitud y longitud (puedes ajustarla a tu ubicación)
    double ref_lat = -38.867787; // Latitud de referencia (Cipolletti)
    double ref_lon = -68.036963; // Longitud de referencia (Cipolletti)

    double latitud;
    double longitud;

    gps_get_lat_lon(tiempo_us, &latitud, &longitud);

    // Si se encontró una trama cercana, mostrar el círculo
    if (latitud != -1.0) {
        // Convertir coordenadas GPS a píxeles
        cv::Point2f pos = gps_to_pixel(latitud, longitud, ref_lat, ref_lon);

	pos.x += 300.0f;
	pos.y += 1000.0f;

        // Dibujar un círculo relleno en la posición calculada
        cv::circle(ventana_completa, pos, 5, cv::Scalar(255, 0, 0), -1); // Círculo rojo de 5 píxeles de radio
        cout << " B " << pos << endl;
    }
}


void mostrar_texto(cv::Mat &ventana_completa, ostringstream &texto, int x, int y)
{

    // Definir el tipo de fuente y el tamaño
    int fontFace = cv::FONT_HERSHEY_SIMPLEX;
    double fontScale = 0.9;
    int thickness = 2;

    // Obtener el tamaño del texto
    int baseline = 0;
    cv::Size textSize = cv::getTextSize(texto.str(), fontFace, fontScale, thickness, &baseline);

    // limpiamos ese sector de la ventana con negro
    cv::Point punto1(x-20, y-30);   // (x1, y1)
    cv::Point punto2(x+textSize.width+20, y+textSize.height+10);  // (x2, y2)
    cv::rectangle(ventana_completa, punto1, punto2, cv::Scalar(0, 0, 0), -1); // -1 relleno 

    // Posicionar el texto 
    cv::Point textOrg(x , y); 

    // Mostrar el texto sobre la imagen
    cv::putText(ventana_completa, texto.str(), textOrg, fontFace, fontScale, cv::Scalar(255, 255, 255), thickness);
}


void mostrar_distancia(cv::Mat &ventana_completa) 
{
    // Formato del texto que vamos a mostrar
    ostringstream texto;
    texto << "distancia: " << distancia << " cm";
    mostrar_texto(ventana_completa, texto, 50, 510);
}


void mostrar_hileras_con_tractor(cv::Mat &ventana_completa, int nro_hilera, int nro_peral) 
{
    int y_pos = 510;  // posicion en Y de la ventana completa
    int alto = 270;
    int ancho = 700;
		      
    static bool hileras_dibujadas = false;
    static cv::Mat imagen_hileras;

    // Parámetros de las hileras
    int num_hileras = 8;  // Número de hileras de perales
    int perales_por_hilera = 33;  // Número de perales por hilera
    int distancia_hilera = 25;  // Espacio entre hileras
    int radio_peral = 3;  // Radio de los círculos que representan los perales
    cv::Scalar color_peral(0, 255, 0);  // Color verde para los perales (B, G, R)

    // Parámetros del tractor
    int radio_tractor = 5;  // Radio del círculo del tractor


    if (!hileras_dibujadas) {
        // Crear la imagen de las hileras solo una vez
        imagen_hileras = cv::Mat::zeros(alto, ancho, CV_8UC3);  // Inicializamos imagen (en negro)

        // Dibujar las hileras de perales en la imagen de las hileras
        for (int i = 0; i < num_hileras; i++) {
            // Calcular la posición de la hilera
            int y_pos = i * distancia_hilera + 50;  // 50 es el desplazamiento inicial

            // Dibujar los perales a lo largo de la hilera
            for (int j = 0; j < perales_por_hilera; j++) {
                int x_pos = j * 20 + 20;  // Espacio entre los perales
                cv::circle(imagen_hileras, cv::Point(x_pos, y_pos), radio_peral, color_peral, -1);
            }
        }

        hileras_dibujadas = true;
    }

    // Copiar las hileras a la ventana
     if (ventana_completa.cols >= imagen_hileras.cols && ventana_completa.rows >= imagen_hileras.rows) {
        imagen_hileras.copyTo(ventana_completa(cv::Rect(0, y_pos, imagen_hileras.cols, imagen_hileras.rows)));
    } else {
        cerr << "Error: ventana con tamaño insuficiente." << endl;
        return;
    }

    // Dibujar el tractor en la hilera y peral indicados
    int tractor_x = nro_peral * 20 + 20;  // x del tractor según el número de peral
    int tractor_y = (nro_hilera * distancia_hilera) - (distancia_hilera/2) + 50 + y_pos;  // y del tractor según la hilera
    cv::Scalar color_tractor;

     if (tractor_color == rojo) 
		color_tractor = cv::Scalar(0, 0, 255);  // Color rojo
	else
		color_tractor = cv::Scalar(0, 255, 0);  // Color verde
							
    cv::circle(ventana_completa, cv::Point(tractor_x, tractor_y), radio_tractor, color_tractor, -1);

    ostringstream texto;
    texto << "grafica de posicionamiento (real-time)";
    mostrar_texto(ventana_completa, texto, 50, y_pos + alto - 10);
}


void mostrar_foto(const cv::Mat& foto_orig, int posicion) 
{
    cv::Mat foto;
    if (foto_orig.channels() != 3) {
        // Si la imagen no tiene 3 canales, convertirla a RGB
        cv::cvtColor(foto_orig, foto, cv::COLOR_GRAY2BGR);  // o cv::COLOR_BGR2RGB dependiendo de la imagen
    } else {
        foto = foto_orig;
    }

    // Definir las posibles 3 imagenes
    static cv::Mat imagen_principal = cv::Mat::zeros(480, 640, CV_8UC3);
    static cv::Mat imagen_pequena1 = cv::Mat::zeros(480, 320, CV_8UC3);
    static cv::Mat imagen_pequena2 = cv::Mat::zeros(480, 320, CV_8UC3);

    // Actualiza la imagen según la posición
    switch (posicion) {
        case 1:
            cv::resize(foto, imagen_principal, imagen_principal.size());
            break;
        case 2:
            cv::resize(foto, imagen_pequena1, imagen_pequena1.size());
            break;
        case 3:
            cv::resize(foto, imagen_pequena2, imagen_pequena2.size());
            break;
        default:
            cerr << "Posición no válida" << endl;
            return;
    }

    // Combina las imágenes en una sola para la ventana
    imagen_principal.copyTo(ventana_completa(cv::Rect(0, 0, 640, 480)));
    imagen_pequena1.copyTo(ventana_completa(cv::Rect(740, 0, 320, 480)));
    imagen_pequena2.copyTo(ventana_completa(cv::Rect(1160, 0, 320, 480)));
}

void mostrar_ventana_completa(void) 
{
    // Llamada a la función para dibujar hileras y tractor en la hilera 3
    mostrar_hileras_con_tractor(ventana_completa, 3, tractor_en_peral);

    mostrar_distancia(ventana_completa);

    mostrar_gps(ventana_completa);

    cv::imshow("Ventana Principal", ventana_completa);
    cv::waitKey(1);  // Para refrescar la ventana sin bloquear
}

void mostrar_init(void)
{
       // ventana principal
        cv::namedWindow("Ventana Principal", cv::WINDOW_NORMAL);
        cv::resizeWindow("Ventana Principal", 900, 700);

        // La ventana completa debe ser de tamaño 1280x480
        ventana_completa = cv::Mat(1000, 1480, CV_8UC3, cv::Scalar(0, 0, 0));

        // leer_mag_out("mag_out.txt");
            // Mostrar la imagen en una ventana
        //cv::imshow("Ventana Principal", ventana_completa);
        //cv::waitKey(0);  // Actualizar la ventana

}
