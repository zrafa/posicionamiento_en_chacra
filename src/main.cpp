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
#include <unistd.h>

// OpenCV
#include <opencv2/opencv.hpp>

using namespace std;

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

int DB = 0;		// ejecutar en modo busqueda

int MARGEN;
int DISTANCIA_ARBOL;
int CONSECUTIVOS;
double UMBRAL_COLOR;
double UMBRAL_GRIS;
int N_ULT_ARBOLES;

int distancia = 0;	/* distancia actual leida desde lidar */
long long tiempo_us = 0;	/* distancia actual leida desde lidar */

frutal ultimos_arboles[20];

extern cv::Ptr<cv::ORB> orb;
extern vector<arbol_db> db;
extern int tractor_color;

struct MagnetometroData {
    double timestamp; // Marca de tiempo en milisegundos
    double x, y, z;   // Valores crudos del magnetómetro
};
std::vector<MagnetometroData> data_magnetometro;



extern cv::Mat ventana_completa;


// Función para leer los datos del archivo magnetometro.txt
std::vector<MagnetometroData> readMagnetometroData(const std::string& filename) {
    std::vector<MagnetometroData> data;
    std::ifstream file(filename);
    std::string line;

    while (std::getline(file, line)) {
        std::istringstream iss(line);
        double valor1, x, y, z, timestamp;
        char ch;

        // Leer el primer valor (valor1) y descartarlo
        iss >> valor1;

        // Leer los valores [x, y, z]
        iss >> ch; // Leer el '['
        iss >> x >> ch >> y >> ch >> z >> ch; // Leer x, y, z y el ']'

        // Leer el timestamp
        iss >> timestamp;

        // Almacenar los datos en el vector
        data.push_back({timestamp, x, y, z});
    }

    return data;
}

// Función para leer el archivo mag_out.txt
void leer_mag_out(const string& filename) {
    ifstream file(filename);  // Abrir el archivo
    if (!file.is_open()) {
        cerr << "Error: No se pudo abrir el archivo " << filename << endl;
        return;
    }

    string line;
    while (getline(file, line)) {  // Leer cada línea del archivo
        double x, y, z;
        stringstream ss(line);  // Convertir la línea en un stream de strings

        // Leer los valores de x, y, z desde el stream
        ss >> x >> y >> z;

        // Mostrar los valores
    // Aplicar el offset de calibración
    double x_offset = -713.4790434;
    double y_offset = -237.35458116;
    double z_offset = 251.28445005;

    x = x - x_offset;
    y = y - y_offset;
    z = z - z_offset;

    // Datos crudos del magnetómetro (ejemplo: [-1450.0, -1387.0, 25.0])
    cv::Mat raw_data = (cv::Mat_<double>(1, 3) << x, y, z);

    // Matriz de transformación (obtenida de la calibración)
    cv::Mat transformation = (cv::Mat_<double>(3, 3) <<
	 0.9644042,  -0.04023857,  0.0,       
	-0.04023857,  1.03963511,  0.0,       
	 0.0,          0.0,          0.9989917);

	cv::Mat calibrated_data = raw_data * transformation.t();
    // Obtener los componentes x, y, z de calibrated_data
    x = calibrated_data.at<double>(0, 0);  // Componente x
    y = calibrated_data.at<double>(0, 1);  // Componente y
    z = calibrated_data.at<double>(0, 2);  // Componente z
						  
	// Dibujar un círculo rojo en el centro del recorte (opcional)
	circle(ventana_completa, cv::Point(x+4000, y+4200), 2, cv::Scalar(0, 0, 255), -1);
        cout << "MAG_OUT x: " << x << ", y: " << y << ", z: " << z << endl;
    }

    file.close();  // Cerrar el archivo
}



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


int tractor_en_peral = 0;






// Estructura para almacenar la información del lidar
struct LidarData {
    int distancia;
    int tiempo_ms;
    long long marca_ms;
    long long marca_us;
};

// Leer los datos del archivo lidar.txt
vector<LidarData> datos_lidar;


// Función para leer los datos del archivo lidar.txt
vector<LidarData> leerDatosLidar(const string& nombreArchivo) {
    vector<LidarData> datos;
    ifstream archivo(nombreArchivo);
    string linea;
    string campo1, campo2, campo3;

    while (getline(archivo, linea)) {
        stringstream ss(linea);
        string token;
        LidarData data;

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




// ------------------------------------------------------------------------------------------------


void encontrar_bordes(const cv::Mat& img, long long marca_tiempo, int *x1, int *x2) 
{
        cv::Mat gray = img;

    // Obtener las dimensiones de la imagen
    int rows = gray.rows;
    int cols = gray.cols;

    // Columna central
    int centralCol = cols / 2;

    distancia = buscarDistanciaCercana(marca_tiempo);

    if (distancia > DISTANCIA_ARBOL) {
        cout << "Distancia lejana: ." << distancia << " MARCA TIEMPO: " << marca_tiempo << endl;
	return;
    }

    // Función para calcular la media de gris entre dos píxeles
    auto diferenciaPixel = [&](int fila, int col1, int col2) {
        return abs(gray.at<uchar>(fila, col1) - gray.at<uchar>(fila, col2));
    };

    // Función para verificar si más del 50% de los píxeles de la columna difieren de la columna adyacente
    auto columnaEsBorde = [&](int col1, int col2) {
        int countDiff = 0;
        for (int i = 0; i < rows; i++) {
            if (diferenciaPixel(i, col1, col2) > UMBRAL_GRIS) {
                countDiff++;
            }
        }
        // Verificar si más del 50% de los píxeles son diferentes
        return (countDiff > (50*rows/100));
    };

    int bordeIzquierdo = -1;
    int bordeDerecho = -1;

    // Buscar borde izquierdo desde la columna central hacia la izquierda
    for (int col = centralCol - 1; col > 0; col--) {
        if (columnaEsBorde(col, centralCol)) {
            bordeIzquierdo = col;
            break;
        }
    }

    // Buscar borde derecho desde la columna central hacia la derecha
    for (int col = centralCol + 1; col < cols - 1; col++) {
        //if (columnaEsBorde(col, col - 1)) {
        if (columnaEsBorde(col, centralCol)) {
            bordeDerecho = col;
            break;
        }
    }

    // Mostrar los resultados
    if (bordeIzquierdo != -1 && bordeDerecho != -1) {
        cout << "Borde izquierdo detectado en x: " << bordeIzquierdo << endl;
        cout << "Borde derecho detectado en x: " << bordeDerecho << endl;
        cout << "Distancia:  " << distancia << endl;
	*x1 = bordeIzquierdo;
	*x2 = bordeDerecho;

	// Dibujar las líneas de los bordes en la imagen
        cv::Mat result;
        cv::cvtColor(gray, result, cv::COLOR_GRAY2BGR);  // Convertir a BGR para dibujar en color
        cv::line(result, cv::Point(bordeIzquierdo, 0), cv::Point(bordeIzquierdo, rows), cv::Scalar(0, 0, 255), 2);  // Línea roja para el borde izquierdo
        cv::line(result, cv::Point(bordeDerecho, 0), cv::Point(bordeDerecho, rows), cv::Scalar(0, 255, 0), 2);  // Línea verde para el borde derecho

    	mostrar_foto(result, 3);
    } else {
        cout << "No se detectaron los bordes del tronco." << endl;
    }

}



// Función para recortar la imagen alrededor del tronco
bool recortar_tronco(const cv::Mat& img, cv::Mat& recortada, const cv::Mat& img_color, cv::Mat& recortada_color) 
{
	double centerX;

	// Convertir a escala de grises
	cv::Mat gray;
	cv::Rect roi2(0, 0, img.cols, img.rows-50);
	gray = img(roi2);
	//gray = img;

	// Aplicar un filtro Gaussiano para reducir el ruido
	cv::Mat blurred;
	GaussianBlur(gray, blurred, cv::Size(5, 5), 2);

	// Analizar continuidad de color en columnas
	vector<int> lowVarianceColumns;
	for (int x = 0; x < gray.cols; ++x) {
	    cv::Mat column = gray.col(x);
	    cv::Scalar mean, stddev;
	    meanStdDev(column, mean, stddev);

	    // Si la desviación estándar es baja, hay poca variación vertical
	    if (stddev[0] < 14) {
       	 // Verificar si la columna contiene tonos de verde en la imagen original de color
       		 cv::Mat colorColumn = img_color.col(x); // Extrae la columna en color
       		 cv::Mat hsvColumn;
       		 cv::cvtColor(colorColumn, hsvColumn, cv::COLOR_BGR2HSV); // Convierte a HSV

       		 bool hasGreen = false;
       		 for (int y = 0; y < hsvColumn.rows; ++y) {
       		     cv::Vec3b hsvPixel = hsvColumn.at<cv::Vec3b>(y, 0);
       		     int hue = hsvPixel[0];

       		     // Verificar si el tono (Hue) está dentro del rango de verde
       		     if (hue >= 35 && hue <= 85) {
       		         hasGreen = true;
       		         break;
       		     }
       		 }

        // Si no tiene verde, lo consideramos un posible tronco
        if (!hasGreen) {
            lowVarianceColumns.push_back(x);
        }
    }
}


	// Agrupar líneas en regiones densas
	sort(lowVarianceColumns.begin(), lowVarianceColumns.end());

	vector<pair<int, int>> regions;
	if (lowVarianceColumns.empty()) {
		cout << "No se encontró un tronco claro color" << endl;
		return false; // Indicar error
	};

		int start = lowVarianceColumns[0];
		int end = start;
		for (size_t i = 1; i < lowVarianceColumns.size(); ++i) {
			if (lowVarianceColumns[i] - end <= 5) {
				end = lowVarianceColumns[i];
			} else {
				regions.push_back(make_pair(start, end));
				start = lowVarianceColumns[i];
				end = start;
			}
		}
		regions.push_back(make_pair(start, end));

		// Encontrar la región con mayor densidad de líneas
		int maxDensity = 0;
		int bestRegionStart = 0;
		int bestRegionEnd = 0;
		for (auto& region : regions) {
			int density = region.second - region.first;
			if (density > maxDensity) {
				maxDensity = density;
				bestRegionStart = region.first;
				bestRegionEnd = region.second;
			}
		}

		// Calcular el centro de la región más densa
		centerX = (bestRegionStart + bestRegionEnd) / 2.0;
		// RAFA para nuevas pruebas if (centerX == 0) {
		if ((centerX == 0) || (centerX < 250) || (centerX > 614) ) {
			cout << "No se encontró un tronco claro" << endl;
			return false;
		}
		cout << " centerx " << centerX << flush ;

	// Ajustar los límites de recorte para mantener 
	// el punto rojo en el centro
	int xLeft = max(0, (int)(centerX - MARGEN));
	int xRight = min(img.cols, (int)(centerX + MARGEN));

	// Ajustar los límites si la región de recorte 
	// se sale del borde de la imagen
	if (xLeft == 0) {
		xRight = min(MARGEN*2, img.cols);
	} else if (xRight == img.cols) {
		xLeft = max(0, img.cols - MARGEN*2);
	}

	// Recortar la imagen
	cv::Rect roi(xLeft, 0, xRight - xLeft, img.rows);
	recortada = img(roi);
	recortada_color = img_color(roi);

	// Dibujar un círculo rojo en el centro del recorte (opcional)
	// circle(recortada, cv::Point(100, recortada.rows / 2), 5, cv::Scalar(0, 0, 255), -1);
	return true;
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


	// ventana principal
	cv::namedWindow("Ventana Principal", cv::WINDOW_NORMAL);
	cv::resizeWindow("Ventana Principal", 900, 700);

	// La ventana completa debe ser de tamaño 1280x480
	ventana_completa = cv::Mat(1000, 1480, CV_8UC3, cv::Scalar(0, 0, 0));

	// leer_mag_out("mag_out.txt");
	    // Mostrar la imagen en una ventana
	cv::imshow("Ventana Principal", ventana_completa);
	cv::waitKey(0);  // Actualizar la ventana


	datos_lidar = leerDatosLidar("lidar.txt");
  	buscar_troncos();

	if (DB)
		db_save("hilera.db");

	return 0;
}

