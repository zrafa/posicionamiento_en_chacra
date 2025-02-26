/*
 * db.cpp : crea, carga y busca en hilera.db
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
#include <db.h>
#include <unistd.h>

// OpenCV
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/ximgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>


using namespace std;

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

// number of training images
/*
const int NIMAGES = 1262;

int BD = 0;		// ejecutar en modo busqueda


int MARGEN;
int DISTANCIA_ARBOL;
int CONSECUTIVOS;

int distancia = 0;	/* distancia actual leida desde lidar */
/*long long tiempo_us = 0;	/* distancia actual leida desde lidar */
#define N_ULT_ARBOLES 4


#include <opencv2/opencv.hpp>




// ----------- FUNCIONES DE AYUDA



void adjustImageToMean(cv::Mat& image, double target_mean) {
    // Calcular el promedio actual de la imagen
    cv::Scalar current_mean_scalar = cv::mean(image);
    double current_mean = current_mean_scalar[0];

    // Calcular la diferencia entre el promedio deseado y el actual
    double shift = target_mean - current_mean;

    // Ajustar la imagen sumando la diferencia
    image.convertTo(image, -1, 1, shift);

    // Recortar los valores para que estén en el rango 0-255
    cv::threshold(image, image, 255, 255, cv::THRESH_TRUNC);
    cv::threshold(image, image, 0, 0, cv::THRESH_TOZERO);
}



// Función para aplicar la Transformada de Retinex multiescala
void applyMSRCR(const cv::Mat& input, cv::Mat& output) {
    cv::Mat logImage;
    cv::Mat retinexImage = cv::Mat::zeros(input.size(), CV_32F);

    // Convertir la imagen a logaritmo para simular la percepción humana de la luz
    cv::Mat floatImage;
    input.convertTo(floatImage, CV_32F, 1.0 / 255.0);  // Convertir a flotante y normalizar
    floatImage += 1.0;  // Evitar logaritmo de cero
    cv::log(floatImage, logImage);

    // Usar filtros gaussianos de diferentes tamaños para realizar Retinex multiescala
    vector<cv::Mat> scales(3);
    cv::GaussianBlur(logImage, scales[0], cv::Size(7, 7), 30);
    cv::GaussianBlur(logImage, scales[1], cv::Size(21, 21), 150);
    cv::GaussianBlur(logImage, scales[2], cv::Size(31, 31), 300);

    // Promediar las escalas de Retinex
    for (size_t i = 0; i < scales.size(); ++i) {
        retinexImage += (logImage - scales[i]) / scales.size();
    }

    // Convertir de vuelta a espacio de valores originales
    cv::exp(retinexImage, retinexImage);
    retinexImage -= 1.0;

    // Normalizar el rango dinámico de la imagen resultante
    cv::normalize(retinexImage, retinexImage, 0, 255, cv::NORM_MINMAX);

    retinexImage.convertTo(output, CV_8U);  // Convertir la imagen de nuevo a 8 bits
}

// ----------- FIN FUNCIONES DE AYUDA

// ------------------------------ BD

vector<arbol_db> db;

extern frutal ultimos_arboles[N_ULT_ARBOLES];

extern cv::Ptr<cv::ORB> orb;



string db_get_foto(int n) 
{
	for (const auto& arbol : db) {
		if (arbol.id == n)
			return arbol.foto;
	}
	return  "empty";
}

struct GPSPosition {
    double latitude;  // Latitud en grados decimales
    double longitude; // Longitud en grados decimales

    GPSPosition(double lat = 0.0, double lon = 0.0) : latitude(lat), longitude(lon) {}
};

double toDecimalDegrees(double degreesMinutes) {
    double degrees = static_cast<int>(degreesMinutes / 100);
    double minutes = degreesMinutes - (degrees * 100);
    return degrees + (minutes / 60.0);
}

double haversineDistance(const GPSPosition& pos1, const GPSPosition& pos2) {
    const double R = 6371.0; // Radio de la Tierra en kilómetros

    double lat1 = pos1.latitude * M_PI / 180.0;
    double lon1 = pos1.longitude * M_PI / 180.0;
    double lat2 = pos2.latitude * M_PI / 180.0;
    double lon2 = pos2.longitude * M_PI / 180.0;

    double dlat = lat2 - lat1;
    double dlon = lon2 - lon1;

    double a = sin(dlat / 2) * sin(dlat / 2) +
               cos(lat1) * cos(lat2) *
               sin(dlon / 2) * sin(dlon / 2);
    double c = 2 * atan2(sqrt(a), sqrt(1 - a));

    return R * c; // Distancia en kilómetros
}

int db_buscar_por_diametro(double diametro_cm, int arbol_id) {
	int cual = -1;
	int max_nro_arboles = 1;
	double min_diametro = 1000.0;

    	for (const auto& arbol : db) {
		if ((abs(arbol.diametro_en_cm - diametro_cm) < min_diametro) &&
		   (abs(arbol.id - arbol_id) <= max_nro_arboles)) {
			min_diametro = abs(arbol.diametro_en_cm - diametro_cm);
			cual = arbol.id;
		}
	}
	return cual;
}

void db_buscar_por_gps(int arbol_id, double latitud, double longitud, int *cual, double *distancia) {
	double min_distance = 1000; 	/* mil metros */
	int max_nro_arboles = 1;

	*cual = -1;
	*distancia = 1000;

    	for (const auto& arbol : db) {
		GPSPosition pos1(latitud, longitud);
		GPSPosition pos2(arbol.latitud, arbol.longitud);


		// Calcular la distancia en METROS entre las dos posiciones
		double distance = haversineDistance(pos1, pos2) * 1000.0;
        	cout << arbol.id << " Distancia GPS " << distance << " " << latitud << " " << arbol.latitud << " " << longitud << " " << arbol.longitud << endl;
		if ((distance < min_distance) &&
		   (abs(arbol.id - arbol_id) <= max_nro_arboles)) {
			min_distance = distance;
			*cual = arbol.id;
		}
	}
	*distancia = min_distance;
}

int db_buscar(const cv::Mat& fotoNueva) {
	cv::Mat descNueva;
    vector<cv::KeyPoint> keypoints;

		    // Aplicar la Transformada de Retinex multiescala
    cv::Mat retinexImage;
    applyMSRCR(fotoNueva, retinexImage);

    // Ajustar el brillo para mejorar la visibilidad
    cv::Mat finalImage;
    retinexImage.convertTo(finalImage, -1, 1.5, 50);  // Incrementar contraste y brillo

    double target_mean = 128.0;

    // Ajustar las imágenes para que tengan el promedio deseado
    adjustImageToMean(finalImage, target_mean);
//    image = finalImage.clone();
    orb->detectAndCompute(finalImage, cv::noArray(), keypoints, descNueva);
    //orb->detectAndCompute(fotoNueva, cv::noArray(), keypoints, descNueva);

    cv::BFMatcher matcher(cv::NORM_HAMMING);
    int mejorId = -1;
    int maxCoincidencias = 0;
    double mejorDistancia = DBL_MAX; // Inicializa a la distancia más alta posible

    for (const auto& arbol : db) {
        int coincidenciasActuales = 0;
        double sumaDistancias = 0.0;

        for (const auto& descBase : arbol.descriptores) {
            if (descBase.rows == 0 || descBase.cols == 0) {
                continue; // Saltar descriptores vacíos
            }

            // Comparar con cada descriptor de la foto nueva
            vector<cv::DMatch> matches;
            matcher.match(descNueva, descBase, matches);

            // Ordenar los matches por distancia
            sort(matches.begin(), matches.end(), [](const cv::DMatch& a, const cv::DMatch& b) {
                return a.distance < b.distance;
            });

            // Filtrar coincidencias utilizando el umbral (threshold) adaptativo
            vector<cv::DMatch> good_matches;
            for (const auto& match : matches) {
                if (match.distance < 60) { // Threshold de ejemplo, ajustar según resultados
                    good_matches.push_back(match);
                    sumaDistancias += match.distance;
                }
            }

            coincidenciasActuales += good_matches.size();
        }

        // Decidir si este árbol es el mejor candidato
        double distanciaMedia = coincidenciasActuales > 0 ? sumaDistancias / coincidenciasActuales : DBL_MAX;
        if (coincidenciasActuales > maxCoincidencias ||
            (coincidenciasActuales == maxCoincidencias && distanciaMedia < mejorDistancia)) {
            maxCoincidencias = coincidenciasActuales;
            mejorDistancia = distanciaMedia;
            mejorId = arbol.id;
        }
    }

    return mejorId;
}







// Función para agregar descriptores ORB de un árbol a la base de datos
void db_add(int id, int diametro_en_px, double diametro_en_cm, double latitud, double longitud, string foto) {
	int i;
    arbol_db arbol;
    arbol.id = id;
    arbol.diametro_en_px = diametro_en_px;
    arbol.diametro_en_cm = diametro_en_cm;
    arbol.latitud = latitud;
    arbol.longitud = longitud;
    arbol.foto = foto;

	for (i=0; i<N_ULT_ARBOLES; i++) {
		cv::Mat desc;
		vector<cv::KeyPoint> keypoints;



		    // Aplicar la Transformada de Retinex multiescala
    cv::Mat retinexImage;
    applyMSRCR(ultimos_arboles[i].image, retinexImage);

    // Ajustar el brillo para mejorar la visibilidad
    cv::Mat finalImage;
    retinexImage.convertTo(finalImage, -1, 1.5, 50);  // Incrementar contraste y brillo

    double target_mean = 128.0;

    // Ajustar las imágenes para que tengan el promedio deseado
    adjustImageToMean(finalImage, target_mean);
//    image = finalImage.clone();


		orb->detectAndCompute(finalImage, cv::noArray(), keypoints, desc);
		//orb->detectAndCompute(ultimos_arboles[i].image, cv::noArray(), keypoints, desc);
		arbol.descriptores.push_back(desc);
    }

    db.push_back(arbol);
}

void db_save(const string& archivo) {
	cv::FileStorage fs(archivo, cv::FileStorage::WRITE);

    fs << "arboles" << "[";
    for (const auto& arbol : db) {
        fs << "{";
        fs << "id" << arbol.id;
        fs << "diametro_en_px" << arbol.diametro_en_px;
        fs << "diametro_en_cm" << arbol.diametro_en_cm;
        fs << "foto" << arbol.foto;
        fs << "latitud" << arbol.latitud;
        fs << "longitud" << arbol.longitud;

        fs << "descriptores" << "[";
        for (const auto& desc : arbol.descriptores) {
            fs << desc;
        }
        fs << "]";
        fs << "}";
    }
    fs << "]";
    fs.release();
}

void db_load(const string& archivo) {
	cv::FileStorage fs(archivo, cv::FileStorage::READ);

	cv::FileNode arboles = fs["arboles"];
    for (const auto& node : arboles) {
        arbol_db arbol;
        node["id"] >> arbol.id;
        node["diametro_en_px"] >> arbol.diametro_en_px;
        node["diametro_en_cm"] >> arbol.diametro_en_cm;
        node["latitud"] >> arbol.latitud;
        node["longitud"] >> arbol.longitud;
        node["foto"] >> arbol.foto;

	cv::FileNode descs = node["descriptores"];
        for (const auto& descNode : descs) {
		cv::Mat descriptor;
            descNode >> descriptor;
            arbol.descriptores.push_back(descriptor);
        }

        db.push_back(arbol);
    }
    fs.release();
    // return baseDatos;
}




// ---------------------------- fin de BD


