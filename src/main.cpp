/*
 * main.cpp: sistema de posicionamiento de un tractor 
 * dentro de las hileras de una chacra 
 * utilizando integración de sensores.
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
#include <lidar.h>
#include <detectar_troncos.h>
#include <unistd.h>

// OpenCV
#include <opencv2/opencv.hpp>

using namespace std;

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

struct MagnetometroData {
    double timestamp; // Marca de tiempo en milisegundos
    double x, y, z;   // Valores crudos del magnetómetro
};
extern std::vector<MagnetometroData> data_magnetometro;

extern cv::Mat ventana_completa;


extern std::vector<MagnetometroData> readMagnetometroData(const std::string& filename);


// Función para leer el archivo de configuración
map<string, int> leer_configuracion(const string& archivo) 
{
	ifstream conf_file(archivo);
        if (!conf_file) {
        	cerr << "Error: config.txt no existe." << endl;
        	exit(1);
        }

	map<string, int> config;
	string linea;

	while (getline(conf_file, linea)) {
		// Saltar líneas vacías o comentarios
		if (linea.empty() || linea[0] == '#') continue;

		// Buscar el signo '='
		size_t pos = linea.find('=');
		if (pos != string::npos) {
			string clave = linea.substr(0, pos);
			string valor = linea.substr(pos + 1);

			// Convertir el valor a entero
			stringstream ss(valor);
			int valorInt;
			ss >> valorInt;

			config[clave] = valorInt;
		}
	}

	conf_file.close();
	return config;
}


// int tractor_en_peral = 0;





// Función para buscar la distancia más cercana dada una marca de tiempo
int buscarDistanciaCercana(long long tiempo_us) {
    int distanciaCercana = -1;
    long long menorDiferencia = numeric_limits<long long>::max();

    for (const auto& dato : datos_lidar) {
        long long diferencia = abs(dato.marca_us - tiempo_us);

        if (diferencia < menorDiferencia) {
            menorDiferencia = diferencia;
            // Aplicar la regla de distancia
	    /*
            if (dato.distancia < 200 && dato.tiempo_ms > 10) {
                distanciaCercana = 400;  // Reemplazar por 400 cm
            } else {
                distanciaCercana = dato.distancia;
            }
	    */
                distanciaCercana = dato.distancia;
        }
    }

    return distanciaCercana;
}







// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

void buscar_troncos();



int main(int argc, char* argv[]) 
{
	// Verifica si es modo "db" o modo "posicionamiento"
	if (argc > 1) {
		if (strcmp(argv[1], "db") == 0) {
			DB = 1;  // ejecutar en modo DB
			cout << " en modo DB " << endl;
		}
	}

	orb = cv::ORB::create(200, 1.01, 3, 65, 2, 4, cv::ORB::HARRIS_SCORE, 45);

	if (!DB) {
		db_load("hilera.db");
		for (int i=0; i<30; i++) {
			cout << db[i].id << " " << db[i].diametro_en_px << " " << db[i].diametro_en_cm << endl;
		}
	}

	std::string filename = "magnetometro.txt";
	data_magnetometro = readMagnetometroData(filename);

	// Leer la configuración
	map<string, int> config = leer_configuracion("config.txt");
	// Acceder a los valores de la configuración
	MARGEN = config["margen"];
	DISTANCIA_ARBOL = config["distancia_arbol"];
	CONSECUTIVOS = config["consecutivos"];
	UMBRAL_COLOR = config["umbral_color"];
	UMBRAL_GRIS = config["umbral_gris"];
	N_ULT_ARBOLES = config["n_ult_arboles"];
	DELAY = config["delay"];


	// inicializar ui y ventana principal
	mostrar_init();

	datos_lidar = lidar_load("lidar.txt");

  	buscar_troncos();

	if (DB)
		db_save("hilera.db");

	return 0;
}

