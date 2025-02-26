/*
 * detectar_troncos.cpp: sistema de posicionamiento de un tractor 
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

struct MagnetometroData {
    double timestamp; // Marca de tiempo en milisegundos
    double x, y, z;   // Valores crudos del magnetómetro
};
std::vector<MagnetometroData> data_magnetometro;


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


cv::Mat ventana_completa;

// Función para mostrar el GPS en la ventana
void mostrar_gps(cv::Mat &ventana_completa) 
{
    // Referencia de latitud y longitud (puedes ajustarla a tu ubicación)
    double ref_lat = -38.867787; // Latitud de referencia (Cipolletti)
    double ref_lon = -68.036963; // Longitud de referencia (Cipolletti)

    double latitud;
    double longitud;

    obtener_gps_latitud_longitud(tiempo_us, &latitud, &longitud);

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

// Función para obtener los valores del magnetómetro más cercanos a la marca de tiempo
void magnetometro_get(double tiempo_ms, double* x, double* y, double* z, double* grados, const std::vector<MagnetometroData>& data) {
    double min_diff = std::numeric_limits<double>::max();
    MagnetometroData closest_data;

    // Buscar el registro más cercano a tiempo_ms
    for (const auto& entry : data) {
        double diff = std::abs(entry.timestamp - tiempo_ms);
        if (diff < min_diff) {
            min_diff = diff;
            closest_data = entry;
        }
    }

    // Aplicar el offset de calibración
    double x_offset = -892.2629067;
    double y_offset = -644.44928645;
    double z_offset = 443.14316061;

    *x = closest_data.x - x_offset;
    *y = closest_data.y - y_offset;
    *z = closest_data.z - z_offset;

    // Calcular los grados (ángulo en el plano XY)
    *grados = std::atan2(*y, *x) * 180 / CV_PI;
    if (*grados < 0) {
        *grados += 360;
    }
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


#define rojo 0
#define verde 1
int tractor_color = rojo;

void mostrar_hileras_con_tractor(cv::Mat &ventana_completa, int num_hileras, int perales_por_hilera,
                               int distancia_hilera, int radio_peral, cv::Scalar color_peral,
                               int radio_tractor, int nro_hilera, int nro_peral) 
{
    int y_pos = 510;  // posicion en Y de la ventana completa
    int alto = 270;
    int ancho = 700;
		      
    static bool hileras_dibujadas = false;  // Variable estática que indica si las hileras ya se dibujaron
    static cv::Mat imagen_hileras;  // Variable estática para guardar la imagen de las hileras

    if (!hileras_dibujadas) {
        // Crear la imagen de las hileras solo una vez
        imagen_hileras = cv::Mat::zeros(alto, ancho, CV_8UC3);  // Inicializamos imagen en negro (o blanco si prefieres)

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
     if (ventana_completa.cols >= imagen_hileras.cols && ventana_completa.rows >= imagen_hileras.rows) {
        imagen_hileras.copyTo(ventana_completa(cv::Rect(0, y_pos, imagen_hileras.cols, imagen_hileras.rows)));
    } else {
        cerr << "Error: La ventana completa no tiene un tamaño suficiente para contener la imagen de las hileras." << endl;
        return;
    }

    // Dibujar el tractor en la hilera y peral indicados
    int tractor_x = nro_peral * 20 + 20;  // Posición x del tractor según el número de peral
    int tractor_y = (nro_hilera * distancia_hilera) - (distancia_hilera/2) + 50 + y_pos;  // Posición y del tractor según la hilera
    cv::Scalar color_tractor;

     if (tractor_color == rojo) 
		color_tractor = cv::Scalar(0, 0, 255);  // Color rojo para el tractor
	else
		color_tractor = cv::Scalar(0, 255, 0);  // Color rojo para el tractor
							//
    cv::circle(ventana_completa, cv::Point(tractor_x, tractor_y), radio_tractor, color_tractor, -1);  // Círculo relleno para el tractor

    ostringstream texto;
    texto << "grafica de posicionamiento (real-time)";
    mostrar_texto(ventana_completa, texto, 50, y_pos + alto - 10);

}




int tractor_en_peral = 0;

// Función para mostrar imágenes en las posiciones deseadas
void mostrar_foto(const cv::Mat& foto_orig, int posicion) 
{
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

    imagen_principal.copyTo(ventana_completa(cv::Rect(0, 0, 640, 480)));

    imagen_pequena1.copyTo(ventana_completa(cv::Rect(740, 0, 320, 480)));

    imagen_pequena2.copyTo(ventana_completa(cv::Rect(1160, 0, 320, 480)));
}

void mostrar_ventana_completa(void) 
{
    // Parámetros de las hileras
    int num_hileras = 8;  // Número de hileras de perales
    int perales_por_hilera = 33;  // Número de perales por hilera
    int distancia_hilera = 25;  // Espacio entre hileras
    int radio_peral = 3;  // Radio de los círculos que representan los perales
    cv::Scalar color_peral(0, 255, 0);  // Color verde para los perales (B, G, R)

    // Parámetros del tractor
    int radio_tractor = 5;  // Radio del círculo del tractor

    // Llamada a la función para dibujar hileras y tractor en la hilera 3, peral 5
    mostrar_hileras_con_tractor(ventana_completa, num_hileras, perales_por_hilera,
                                distancia_hilera, radio_peral, color_peral,
                                radio_tractor, 3, tractor_en_peral);

    mostrar_distancia(ventana_completa);

    mostrar_gps(ventana_completa);
    cv::imshow("Ventana Principal", ventana_completa);
    cv::waitKey(1);  // Para refrescar la ventana sin bloquear
}



//  ---------------- DIAMETRO
//
// Calcular el diametro a partir de las muestras que quedan dentro del desvío
// estándar
double diametro_medio(const vector<double>& datos) 
{
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





// ---------------------- ENCONTRAR TRONCO

// ----------------------- FUNCIONES DE AYUDA



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

	datos_lidar = leerDatosLidar("lidar.txt");
  	buscar_troncos();

	if (DB)
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
//   en la DB:
//        NRO de arbol en la hilera
//        diametro
//        los 4 fingerprints para el mismo arbol
//
//   Cuando se busque un arbol en la DB, solo existiran datos (en la DB) de algunos arboles de la hilera. 
//   Arboles interesantes (los delgados o con diametros que fluctuaron no estarán en la DB)
//
//   Entonces el algoritmo de posicionamiento será así:
//       - por un lado, cuando se detecte un arbol en la foto 3 veces, con distancia acorde,
//         se contará + 1 (luego tiene que venir un periodo de "no distancia", para volver a contar un arbol
//         Lo anterior intentará posicionarse "contando" los arboles en la hilera.
//       - en paralelo, cuando el arbol parezca interesante (diametro parejo, distancia acorde, etc).
//         se intentará buscar ese arbol en la DB (por diametro, orb descriptors).
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
                cout << " Tiempo transcurrido foto : " << numero << " " << ii-1 << "  " << duration.count() << " segundos" << endl; 
		// Inicia un nuevo cronometro
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
		mostrar_ventana_completa();
    
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
			if (DB) {
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
    				ostringstream texto;
    				texto << "gps lat=" <<  std::fixed << std::setprecision(6) << latitud << " lon=" << longitud;
    				mostrar_texto(ventana_completa, texto, 700, 600);

				if (! (distancias_dispares(distancias) || (diametros_dispares(diametros)))) {
					double diametro_en_cm = diametro_medio(diametros);
					int cual_arbol; double cual_diametro;
					db_buscar_por_diametro(diametro_en_cm, arbol, &cual_arbol, &cual_diametro);
					cout << arbol << " arbol por diametro FINAL es: " << cual_arbol << " " << cual_diametro << endl;
    					ostringstream texto;
    					texto << "diametro=" << diametro_en_cm << " cm           ";
    					mostrar_texto(ventana_completa, texto, 700, 550);
				}

    // Ejemplo de uso
    double tiempo_ms = tiempo_us / 1000; // Marca de tiempo en milisegundos
    double x, y, z, grados;
    magnetometro_get(tiempo_ms, &x, &y, &z, &grados, data_magnetometro);
    std::cout << "MAGNE x: " << x << ", y: " << y << ", z: " << z << ", grados: " << grados << std::endl;

				int cant_arboles = 50;
				int arbol_en_db[50] = {0};
				for (i=0; i<N_ULT_ARBOLES;i++) {
					int cual = db_buscar(ultimos_arboles[i].image);

					cout << arbol << " arbol orb es: " << cual << " " << ss.str() << " - " << db_get_foto(cual) << endl;
					arbol_en_db[cual]++;
				}
				for (i=0; i<cant_arboles;i++) {
					if (arbol_en_db[i] >= (N_ULT_ARBOLES/2)) {
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

