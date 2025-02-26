/**
 * File: Demo.cpp
 * Date: November 2011
 * Author: Dorian Galvez-Lopez
 * Description: demo application of DBoW2
 * License: see the LICENSE.txt file
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

#include <opencv2/opencv.hpp>

using namespace std;

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

// number of training images
const int NIMAGES = 1262;

int BD = 0;		// ejecutar en modo busqueda

int MARGEN;
int DISTANCIA_ARBOL;
int CONSECUTIVOS;
double UMBRAL_COLOR;
double UMBRAL_GRIS;

int distancia = 0;	/* distancia actual leida desde lidar */
long long tiempo_us = 0;	/* distancia actual leida desde lidar */
#define N_ULT_ARBOLES 4


frutal ultimos_arboles[N_ULT_ARBOLES];

extern cv::Ptr<cv::ORB> orb;



extern vector<arbol_db> db;



// Estructura para almacenar los datos GPS
struct GPS_data {
    long long timestamp_us;
    double latitude;
    double longitude;
};

// Función para convertir coordenadas GPS a píxeles
cv::Point2f gpsToPixel(double latitude, double longitude, double ref_lat, double ref_lon) {
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

void obtener_gps_latitud_longitud (long long tiempo_us, double *latitud, double *longitud) {

    // Abrir el archivo gps.txt
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

    // Cerrar el archivo
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

// Función para mostrar el GPS en la ventana
void mostrar_gps(cv::Mat &ventana_completa) {
    // Referencia de latitud y longitud (puedes ajustarla a tu ubicación)
    double ref_lat = -38.867787; // Latitud de referencia (Cipolletti)
    double ref_lon = -68.036963; // Longitud de referencia (Cipolletti)

    double latitud;
    double longitud;
    obtener_gps_latitud_longitud(tiempo_us, &latitud, &longitud);

    // Si se encontró una trama cercana, mostrar el círculo
    if (latitud != -1.0) {
        // Convertir coordenadas GPS a píxeles
        cv::Point2f pos = gpsToPixel(latitud, longitud, ref_lat, ref_lon);

	pos.x += 800.0f;
	pos.y += 600.0f;

        // Dibujar un círculo relleno en la posición calculada
        cv::circle(ventana_completa, pos, 5, cv::Scalar(255, 0, 0), -1); // Círculo rojo de 5 píxeles de radio
        cout << " B " << pos << endl;
    }
}











// Función para leer el archivo de configuración
map<string, int> leerConfiguracion(const string& archivo) {
    ifstream archivoConfig(archivo);
        if (!archivoConfig) {
        	cerr << "Error: config.txt no existe." << endl;
        	exit(1);
        }

    map<string, int> config;
    string linea;

    while (getline(archivoConfig, linea)) {
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

    archivoConfig.close();
    return config;
}

void mostrar_distancia(cv::Mat &ventana_completa) 
{

    // Formato del texto que vamos a mostrar
    ostringstream texto;
    texto << "distancia: " << distancia << " cm";

    // Definir el tipo de fuente y el tamaño
    int fontFace = cv::FONT_HERSHEY_SIMPLEX;
    double fontScale = 1;
    int thickness = 2;

    // Obtener el tamaño del texto
    int baseline = 0;
    cv::Size textSize = cv::getTextSize(texto.str(), fontFace, fontScale, thickness, &baseline);

    // Posicionar el texto en la parte superior izquierda, asegurando que "cm" y el número estén alineados
    //cv::Point textOrg(ventana_completa.cols - textSize.width - 10, 500); // Ajuste de la posición X para alinear el texto
    cv::Point textOrg(30 , 520); // Ajuste de la posición X para alinear el texto

    // Mostrar el texto sobre la imagen
    cv::putText(ventana_completa, texto.str(), textOrg, fontFace, fontScale, cv::Scalar(255, 255, 255), thickness);

    // Mostrar la imagen en una ventana
   // cv::imshow("Ventana Principal", ventana_completa);
    // Esperar hasta que el usuario presione una tecla
    //cv::waitKey(1);
}


#define rojo 0
#define verde 1
int tractor_color = rojo;

void dibujarHilerasConTractor(cv::Mat &ventana_completa, int num_hileras, int perales_por_hilera,
                               int distancia_hilera, int radio_peral, cv::Scalar color_peral,
                               int radio_tractor, int nro_hilera, int nro_peral) {
    static bool hileras_dibujadas = false;  // Variable estática que indica si las hileras ya se dibujaron
    static cv::Mat imagen_hileras;  // Variable estática para guardar la imagen de las hileras

    if (!hileras_dibujadas) {
        // Crear la imagen de las hileras solo una vez
        imagen_hileras = cv::Mat::zeros(250, 700, CV_8UC3);  // Inicializamos imagen en negro (o blanco si prefieres)

        // Dibujar las hileras de perales en la imagen de las hileras
        for (int i = 0; i < num_hileras; i++) {
            // Calcular la posición de la hilera
            int y_pos = i * distancia_hilera + 50;  // 50 es el desplazamiento inicial

            // Dibujar los perales a lo largo de la hilera
            for (int j = 0; j < perales_por_hilera; j++) {
                int x_pos = j * 20 + 20;  // Espacio entre los perales
                cv::circle(imagen_hileras, cv::Point(x_pos, y_pos), radio_peral, color_peral, -1);  // Dibujar círculo relleno
            }
        }

        // Marcar que las hileras ya fueron dibujadas
        hileras_dibujadas = true;
    }

    // Copiar las hileras a la ventana de salida
    //imagen_hileras.copyTo(ventana_completa);  // Copiar el contenido de imagen_hileras a ventana_completa
    //imagen_hileras.copyTo(ventana_completa(cv::Rect(0, 400, 640, 480)));
     if (ventana_completa.cols >= imagen_hileras.cols && ventana_completa.rows >= imagen_hileras.rows) {
        // Verificar que la subregión seleccionada en ventana_completa tiene un tamaño adecuado
        imagen_hileras.copyTo(ventana_completa(cv::Rect(0, 500, imagen_hileras.cols, imagen_hileras.rows)));
    } else {
        cerr << "Error: La ventana completa no tiene un tamaño suficiente para contener la imagen de las hileras." << endl;
        return;
    }


    // Dibujar el tractor en la hilera y peral indicados
    int tractor_x = nro_peral * 20 + 20;  // Posición x del tractor según el número de peral
    int tractor_y = (nro_hilera * distancia_hilera) - (distancia_hilera/2) + 50 + 500;  // Posición y del tractor según la hilera
    cv::Scalar color_tractor;

     if (tractor_color == rojo) 
		color_tractor = cv::Scalar(0, 0, 255);  // Color rojo para el tractor
	else
		color_tractor = cv::Scalar(0, 255, 0);  // Color rojo para el tractor
    cv::circle(ventana_completa, cv::Point(tractor_x, tractor_y), radio_tractor, color_tractor, -1);  // Círculo relleno para el tractor

    // Mostrar la imagen
  //  cv::imshow("Ventana Principal", ventana_completa);
   // cv::waitKey(1);  // Esperar a que se cierre la ventana
		     //
		     //
}





int tractor_en_peral = 0;

// Función para mostrar imágenes en las posiciones deseadas
void mostrar_foto(const cv::Mat& foto_orig, int posicion) {
    // Crea la ventana si no existe
    static bool ventana_creada = false;
    if (!ventana_creada) {
        cv::namedWindow("Ventana Principal", cv::WINDOW_NORMAL);
        ventana_creada = true;
    }

        cv::Mat foto;
    if (foto_orig.channels() != 3) {
        // Si la imagen no tiene 3 canales, convertirla a RGB
        cv::cvtColor(foto_orig, foto, cv::COLOR_GRAY2BGR);  // o cv::COLOR_BGR2RGB dependiendo de la imagen
    } else {
        foto = foto_orig;
    }


    // Definir la imagen principal (640x480) y las pequeñas (320x480)
    static cv::Mat imagen_principal = cv::Mat::zeros(480, 640, CV_8UC3);  // Imagen principal (640x480)
    static cv::Mat imagen_pequena1 = cv::Mat::zeros(480, 320, CV_8UC3);  // Imagen pequeña 1 (320x480)
    static cv::Mat imagen_pequena2 = cv::Mat::zeros(480, 320, CV_8UC3);  // Imagen pequeña 2 (320x480)

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
    // La ventana completa debe ser de tamaño 1280x480
    //cv::Mat ventana_completa(480, 1480, CV_8UC3, cv::Scalar(0, 0, 0));
    cv::Mat ventana_completa(800, 1480, CV_8UC3, cv::Scalar(0, 0, 0));

    imagen_principal.copyTo(ventana_completa(cv::Rect(0, 0, 640, 480)));

    imagen_pequena1.copyTo(ventana_completa(cv::Rect(740, 0, 320, 480)));

    imagen_pequena2.copyTo(ventana_completa(cv::Rect(1160, 0, 320, 480)));



		     //
		     //


    // Parámetros de las hileras
    int num_hileras = 10;  // Número de hileras de perales
    int perales_por_hilera = 33;  // Número de perales por hilera
    int distancia_hilera = 25;  // Espacio entre hileras
    int radio_peral = 3;  // Radio de los círculos que representan los perales
    cv::Scalar color_peral(0, 255, 0);  // Color verde para los perales (B, G, R)

    // Parámetros del tractor
    int radio_tractor = 5;  // Radio del círculo del tractor

    // Crear una ventana para mostrar la imagen
    //cv::Mat ventana_completa(500, 600, CV_8UC3);  // Imagen de 500x600 px

    // Llamada a la función para dibujar hileras y tractor en la hilera 3, peral 5
    dibujarHilerasConTractor(ventana_completa, num_hileras, perales_por_hilera,
                             distancia_hilera, radio_peral, color_peral,
                             radio_tractor, 3, tractor_en_peral);

    mostrar_distancia(ventana_completa);

    // Mostrar la ventana
    //cv::imshow("Ventana Principal", ventana_completa);
    //cv::waitKey(1);  // Para refrescar la ventana sin bloquear
    mostrar_gps(ventana_completa);
    cv::imshow("Ventana Principal", ventana_completa);
    cv::waitKey(1);  // Para refrescar la ventana sin bloquear
}






//  ---------------- DIAMETRO
//
// Calcular el diametro a partir de las muestras que quedan dentro del desvío
// estándar
double diametro_medio(const vector<double>& datos) {
    // Calcular la media aritmética
    double suma = 0;
    for (double valor : datos) {
        suma += valor;
    }
    double media = suma / datos.size();

    // Calcular la varianza
    double sumaVarianza = 0;
    for (double valor : datos) {
        sumaVarianza += pow(valor - media, 2);
    }
    double varianza = sumaVarianza / datos.size();

    // Calcular el desvío estándar
    double desvioEstandar = sqrt(varianza);

    // Filtrar muestras dentro del desvío estándar
    vector<double> dentroDelDesvio;
    for (double valor : datos) {
        if (valor >= media - desvioEstandar && valor <= media + desvioEstandar) {
            dentroDelDesvio.push_back(valor);
        }
    }

    // Calcular la mediana de las muestras dentro del desvío estándar
    sort(dentroDelDesvio.begin(), dentroDelDesvio.end());
    double mediana;
    size_t n = dentroDelDesvio.size();
    if (n % 2 == 0) {
        mediana = (dentroDelDesvio[n / 2 - 1] + dentroDelDesvio[n / 2]) / 2;
    } else {
        mediana = dentroDelDesvio[n / 2];
    }

    return mediana;
}





// ---------------------- PINTAR TRONCO 

// Función para calcular la media de un parche central de 10x10 píxeles
double calcularMediaParcheCentral(const cv::Mat& gray, int centroX, int patchSize) {
    int rows = gray.rows;
    // int cols = gray.cols;

    int centralRow = rows / 2;
    // int centralCol = cols / 2;
    int centralCol = centroX;

    double sum = 0.0;
    int count = 0;

    for (int i = -patchSize/2; i < patchSize/2; i++) {
        for (int j = -patchSize/2; j < patchSize/2; j++) {
            sum += gray.at<uchar>(centralRow + i, centralCol + j);
            count++;
        }
    }

    return sum / count;
}



// ---------------------- ENCONTRAR TRONCO

// ----------------------- FUNCIONES DE AYUDA



// ---------------------------- BD
// ---------------------------- fin de BD



void ult_arboles_init(void )
{
	int i;
	for (i=0; i<N_ULT_ARBOLES; i++){
		ultimos_arboles[i].nro_arbol = -1;
		ultimos_arboles[i].distancia = -1;
		ultimos_arboles[i].diametro = -1;
		ultimos_arboles[i].latitud = -1;
		ultimos_arboles[i].longitud = -1;
		ultimos_arboles[i].x1 = -1;
		ultimos_arboles[i].x2 = -1;
	}
}



// Estructura para almacenar la información del lidar
struct LidarData {
    int distancia;
    int tiempo_ms;
    long long marca_ms;
    long long marca_us;
};

    // Leer los datos del archivo lidar.txt
	vector<LidarData> datosLidar;


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

    for (const auto& dato : datosLidar) {
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

	//pintarTronco(result, (bordeDerecho-bordeIzquierdo)/2+bordeIzquierdo, 15);
        // Mostrar la imagen con los bordes detectados
        // RAFA cv::imshow("Bordes del tronco detectados", result);
        // RAFA cv::waitKey(0);
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
    // Verifica si el número de argumentos es mayor que 1
    if (argc > 1) {
        // Compara el primer argumento con "bd"
        if (strcmp(argv[1], "bd") == 0) {
            BD = 1;  // ejecutar en modo BD
		    cout << " en modo BD " << endl;
        }
    }

    orb = cv::ORB::create(200, 1.01, 3, 65, 2, 4, cv::ORB::HARRIS_SCORE, 45);

    if (!BD) {
	    db_load("hilera.db");
	    int i;
	    for (i=0; i<30; i++) {
		    cout << db[i].id << " " << db[i].diametro_en_px << " " << db[i].diametro_en_cm << endl;
	    }

    }

    // Leer la configuración desde el archivo
    map<string, int> config = leerConfiguracion("config.txt");
    // Acceder a los valores de la configuración
    MARGEN = config["margen"];
    DISTANCIA_ARBOL = config["distancia_arbol"];
    CONSECUTIVOS = config["consecutivos"];
    UMBRAL_COLOR = config["umbral_color"];
    UMBRAL_GRIS = config["umbral_gris"];


    // Inicializar la ventana principal
    cv::namedWindow("Ventana Principal", cv::WINDOW_NORMAL);
    //cv::resizeWindow("Ventana Principal", 1280, 480);
    cv::resizeWindow("Ventana Principal", 800, 600);




	datosLidar = leerDatosLidar("lidar.txt");
  	buscar_troncos();

    if (BD)
	db_save("hilera.db");

  return 0;
}

// IDEAS:
//   tener un arreglo de 4 arboles con los datos:
//      - nro de arbol en la hilera
//      - distancia
//      - diametro
//      - tal vez orb descriptors
//
//   Entonces, si se detectó un tronco al menos 3 veces con distancia acorde, entonces
//   registrar los 4 siguientes arboles (siempre que cumplan la condiciones:
//   - hay tronco en la foto
//   - la distancia es acorde
//   - tambien registrar el diametro en el arreglo.
//
//   Si los diametros coinciden y son "mas o menos interesantes (no muy delgados)", registrar
//   en la BD:
//        NRO de arbol en la hilera
//        diametro
//        los 4 fingerprints para el mismo arbol
//
//   Cuando se busque un arbol en la BD, solo existiran datos (en la BD) de algunos arboles de la hilera. 
//   Arboles interesantes (los delgados o con diametros que fluctuaron no estarán en la BD)
//
//   Entonces el algoritmo de posicionamiento será así:
//       - por un lado, cuando se detecte un arbol en la foto 3 veces, con distancia acorde,
//         se contará + 1 (luego tiene que venir un periodo de "no distancia", para volver a contar un arbol
//         Lo anterior intentará posicionarse "contando" los arboles en la hilera.
//       - en paralelo, cuando el arbol parezca interesante (diametro parejo, distancia acorde, etc).
//         se intentará buscar ese arbol en la BD (por diametro, orb descriptors).
//
//       
// ----------------------------------------------------------------------------

int diametros_dispares(const vector<double>& datos) 
{
	int media = 0;
	int i;
	for (i=0; i<N_ULT_ARBOLES; i++) {
		media += datos[i];
		cout << datos[i] << " a " << endl;
	}
	media = media / N_ULT_ARBOLES;
		cout << media << " media diam a " << endl;
	for (i=0; i<N_ULT_ARBOLES; i++) {
		if ((datos[i] < (media-10)) || 
		    (datos[i] > (media+10))) {
				cout << datos[i] << "   a " << (media-10) << " " << (media+10) << endl;

			return 1;
		}
	}
	return 0;
}

int distancias_dispares(const vector<double>& datos) 
{
	int media = 0;
	int i;
	for (i=0; i<N_ULT_ARBOLES; i++) {
		media += datos[i];
		cout << datos[i] << " a " << endl;
	}
	media = media / N_ULT_ARBOLES;
		cout << media << " media a " << endl;
	for (i=0; i<N_ULT_ARBOLES; i++) {
		if ((datos[i] < (media-2)) || 
		    (datos[i] > (media+2))) {
				cout << datos[i] << "   a " << (media-2) << " " << (media+2) << endl;
			return 1;
		}
	}
	return 0;
}

void buscar_troncos()
{
	int tractor_en_hilera = 3;

	ifstream archivo("listado.txt");
	if (!archivo.is_open()) {
		cerr << "Error al abrir el archivo." << endl;
		 exit (1);
	}

	// Leer el número de fotos 
	int numero;
	archivo >> numero;

	// Ignorar el resto de la primera línea (por si hay más datos)
	archivo.ignore(numeric_limits<streamsize>::max(), '\n');

	// vemos si podemos encontrar el arbol
	int x1, x2;  // posible borde de un arbol
	int arbol = 0;  // nro de arbol en la hilera
	int total = 0;
  	int i;
	chrono::time_point<chrono::high_resolution_clock> start;
	int ii;
	for (ii=0; ii<numero; ii++) {

                // Captura el tiempo final
                auto end = chrono::high_resolution_clock::now();
                // Calcula la duración
                chrono::duration<double> duration = end - start;
                // Imprime la duración en segundos
                cout << " Tiempo transcurrido foto : " << numero << " " << ii-1 << "  " << duration.count() << " segundos" << endl; // Inicia un nuevo cronometro
                start = chrono::high_resolution_clock::now();


		// Leer las líneas restantes y procesarlas
		string linea;
		getline(archivo, linea);
		stringstream ss(linea);

		cv::Mat image = cv::imread(ss.str(), cv::IMREAD_GRAYSCALE);
		cv::Mat image_color = cv::imread(ss.str(), cv::IMREAD_COLOR);

		if (image.empty()) {
			cerr << "No se pudo cargar la imagen." << endl;
			exit(1);
		}


		string nombreArchivo = ss.str();
		// Encontrar la posición del punto para eliminar la extensión
		size_t pos = nombreArchivo.find(".jpg");
		// Extraer la parte del nombre sin la extensión
		string marcaTiempoStr = nombreArchivo.substr(0, pos);
		// Convertir el string a long long
		long long marcaTiempo = stoll(marcaTiempoStr);

			tiempo_us = marcaTiempo;

		mostrar_foto(image_color, 1);
		// usleep(50000);
    
		distancia = buscarDistanciaCercana(marcaTiempo);
		if (distancia > DISTANCIA_ARBOL) {
			total = 0;
			continue;
		}
		if (!recortar_tronco(image, image, image_color, image_color)) {
			continue;
		}
		if (total == N_ULT_ARBOLES)
			continue;

		ultimos_arboles[total].nro_arbol = arbol;
		ultimos_arboles[total].distancia = buscarDistanciaCercana(marcaTiempo);
		encontrar_bordes(image, marcaTiempo, &x1, &x2);
		ultimos_arboles[total].x1 = x1;
		ultimos_arboles[total].x2 = x2;
		ultimos_arboles[total].diametro = x2-x1;
		cv::Rect roi(x1, 0, x2-x1, image.rows);
		ultimos_arboles[total].image = image.clone();


		if (total == (CONSECUTIVOS-1)) {
			arbol++;
                	cout << " :tronco detectado. " << arbol << " " << total << " " << ss.str(); 
			encontrar_bordes(image, marcaTiempo, &x1, &x2);

			cv::Mat image2 = cv::imread(ss.str(), cv::IMREAD_GRAYSCALE);
			mostrar_foto(ultimos_arboles[total].image, 2);
		

			tractor_en_peral++;
			tractor_color = rojo;
		}
		if (total == (N_ULT_ARBOLES-1)) {
			vector<double> diametros;
			vector<double> distancias;
			double tmp = 0.0;
			for (i=0; i<N_ULT_ARBOLES; i++) {
                		cout << " diametro: " << ultimos_arboles[i].diametro << "  distancia: " << ultimos_arboles[i].distancia << " relacion: " << ((double)ultimos_arboles[i].diametro / (double)ultimos_arboles[i].distancia) << endl;
                		tmp = (((double)ultimos_arboles[i].diametro / (double)ultimos_arboles[i].distancia) * 100.0) / PIXELES_X_CM;
				diametros.push_back(tmp);
				distancias.push_back(ultimos_arboles[i].distancia);
			}
			if (BD) {
				if (distancias_dispares(distancias) || (diametros_dispares(diametros))) {
					cout << arbol << " :distancias dispares " << endl;
					double latitud; double longitud;
					obtener_gps_latitud_longitud(tiempo_us, &latitud, &longitud);
					db_add(arbol, -1, -1.0, latitud, longitud, ss.str());
				} else {
    					double diametro_en_cm = diametro_medio(diametros);
               				cout << arbol << " :diametro medio en cm (sin distancia): . " << diametro_en_cm << endl;
					double latitud; double longitud;
					obtener_gps_latitud_longitud(tiempo_us, &latitud, &longitud);
					db_add(arbol, (int)diametro_en_cm * (int)PIXELES_X_CM, diametro_en_cm, latitud, longitud, ss.str());
				}
			} else {
				double latitud; double longitud;
				obtener_gps_latitud_longitud(tiempo_us, &latitud, &longitud);
				int cual; double distancia;
				db_buscar_por_gps(arbol, latitud, longitud, &cual, &distancia);
				cout << arbol << " arbol por GPS FINAL es: " << cual <<  " distancia: " << distancia << endl;

				if (! (distancias_dispares(distancias) || (diametros_dispares(diametros)))) {
					double diametro_en_cm = diametro_medio(diametros);
					int cual_diametro;
					cual_diametro = db_buscar_por_diametro(diametro_en_cm, arbol);
					cout << arbol << " arbol por diametro FINAL es: " << cual_diametro << endl;
				}

				int cant_arboles = 50;
				int arbol_en_bd[50] = {0};
				for (i=0; i<N_ULT_ARBOLES;i++) {
					int cual = db_buscar(ultimos_arboles[i].image);

					cout << arbol << " arbol orb es: " << cual << " " << ss.str() << " - " << db_get_foto(cual) << endl;
					arbol_en_bd[cual]++;
				}
				for (i=0; i<cant_arboles;i++) {
					if (arbol_en_bd[i] >= (N_ULT_ARBOLES/2)) {
						cout << arbol << " arbol orb FINAL es: " << i << endl;
						tractor_en_peral = i;
						tractor_color = verde;
						break;
					} 
				}
			}
		}
		total++;

	}
}

// ----------------------------------------------------------------------------

