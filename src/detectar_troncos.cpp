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
    double x_offset = -713.4790434;
    double y_offset = -237.35458116;
    double z_offset = 251.28445005;

    *x = closest_data.x - x_offset;
    *y = closest_data.y - y_offset;
    *z = closest_data.z - z_offset;

    // Datos crudos del magnetómetro (ejemplo: [-1450.0, -1387.0, 25.0])
    cv::Mat raw_data = (cv::Mat_<double>(1, 3) << *x, *y, *z);

    // Matriz de transformación (obtenida de la calibración)
    cv::Mat transformation = (cv::Mat_<double>(3, 3) <<
	 0.9644042,  -0.04023857,  0.0,       
	-0.04023857,  1.03963511,  0.0,       
	 0.0,          0.0,          0.9989917);

	cv::Mat calibrated_data = raw_data * transformation.t();
    // Obtener los componentes x, y, z de calibrated_data
    *x = calibrated_data.at<double>(0, 0);  // Componente x
    *y = calibrated_data.at<double>(0, 1);  // Componente y
    *z = calibrated_data.at<double>(0, 2);  // Componente z
						  
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


int tractor_en_peral = 0;



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
    // Ejemplo de uso
    double tiempo_ms = tiempo_us / 1000; // Marca de tiempo en milisegundos
    double x, y, z, grados;
    magnetometro_get(tiempo_ms, &x, &y, &z, &grados, data_magnetometro);

    std::cout << "MAGNE x: " << x << ", y: " << y << ", z: " << z << ", grados: " << grados << std::endl;
    mostrar_orientacion(grados);
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
    				texto << "gps lat=" << std::fixed << std::setprecision(8) << latitud << " lon=" << longitud;
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

