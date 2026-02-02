#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <vector>
#include <string>

#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <regex>
#include <algorithm>
#include <iostream>

struct Arbol {
    int id;
    int diametro_en_px;
    double diametro_en_cm;
    std::string foto;
    double latitud;
    double longitud;
    std::vector<std::string> descriptores_raw;  // texto YAML de descriptores
};

std::vector<Arbol> arboles;  // Base de datos cargada en memoria

void db_load() {
    std::ifstream file("hilera.db");
    if (!file.is_open()) {
        std::cerr << "No se pudo abrir hilera.db\n";
        return;
    }

    std::string line;
    Arbol actual;
    bool leyendo_descriptores = false;

    while (std::getline(file, line)) {
        if (line.find("id:") != std::string::npos && !leyendo_descriptores) {
            if (actual.id > 0)
                arboles.push_back(actual);
            actual = Arbol();  // reset
            actual.id = std::stoi(line.substr(line.find(":") + 1));
        } else if (line.find("diametro_en_px:") != std::string::npos) {
            actual.diametro_en_px = std::stoi(line.substr(line.find(":") + 1));
        } else if (line.find("diametro_en_cm:") != std::string::npos) {
            actual.diametro_en_cm = std::stod(line.substr(line.find(":") + 1));
        } else if (line.find("foto:") != std::string::npos) {
            size_t start = line.find("\"") + 1;
            size_t end = line.rfind("\"");
            actual.foto = line.substr(start, end - start);
        } else if (line.find("latitud:") != std::string::npos) {
            actual.latitud = std::stod(line.substr(line.find(":") + 1));
        } else if (line.find("longitud:") != std::string::npos) {
            actual.longitud = std::stod(line.substr(line.find(":") + 1));
        } else if (line.find("descriptores:") != std::string::npos) {
            leyendo_descriptores = true;
            actual.descriptores_raw.clear();
        } else if (leyendo_descriptores) {
            if (line.find("id:") != std::string::npos) {
                leyendo_descriptores = false;
                arboles.push_back(actual);
                actual = Arbol();  // reinicio
                actual.id = std::stoi(line.substr(line.find(":") + 1));
            } else {
                actual.descriptores_raw.push_back(line);
            }
        }
    }

    // Agregar el último si no se agregó aún
    if (actual.id > 0)
        arboles.push_back(actual);

    file.close();
}

bool db_check(const std::string& nombre_jpg) {
    long long nuevo_ts = std::stoll(nombre_jpg.substr(0, nombre_jpg.find(".")));

    for (const auto& a : arboles) {
        std::string base = a.foto.substr(0, a.foto.find("."));
        long long ts = std::stoll(base);
        if (std::abs(ts - nuevo_ts) <= 2000000) {
            return true;  // Ya existe una entrada cercana
        }
    }

    return false;  // No hay ninguna en ±2 segundos
}


void db_add(const std::string& nombre_jpg) {
	std::cout << " nombre " << nombre_jpg << std::endl;
    long long nuevo_ts = std::stoll(nombre_jpg.substr(0, nombre_jpg.find(".")));

    for (const auto& a : arboles) {
        std::string base = a.foto.substr(0, a.foto.find("."));
        long long ts = std::stoll(base);
        if (std::abs(ts - nuevo_ts) <= 2000000) {
            return;  // ya hay uno cercano
        }
    }

    // Determinar posición de inserción ordenada por timestamp
    int pos = 0;
    while (pos < arboles.size()) {
        long long ts = std::stoll(arboles[pos].foto.substr(0, arboles[pos].foto.find(".")));
        if (nuevo_ts < ts) break;
        pos++;
    }

        // Insertar nuevo Arbol
    Arbol nuevo;
    nuevo.foto = nombre_jpg;
    nuevo.diametro_en_px = -1;
    nuevo.diametro_en_cm = -1;
    nuevo.latitud = -1;
    nuevo.longitud = -1;
    nuevo.descriptores_raw = {
        "         - !!opencv-matrix",
        "            rows: 0",
        "            cols: 0",
        "            dt: u",
        "            data: []"
    };

    arboles.insert(arboles.begin() + pos, nuevo);

    // Reasignar IDs secuenciales desde 1
    for (int i = 0; i < arboles.size(); ++i) {
        arboles[i].id = i + 1;
    }

}

void db_save() {
    std::ofstream file("hilera.db");
    if (!file.is_open()) {
        std::cerr << "No se pudo escribir hilera.db\n";
        return;
    }

    file << "%YAML:1.0\n";
    file << "---\n";
    file << "arboles:\n";

    for (const auto& a : arboles) {
        file << "   -\n";
        file << "      id: " << a.id << "\n";
        file << "      diametro_en_px: " << a.diametro_en_px << "\n";
        file << "      diametro_en_cm: " << a.diametro_en_cm << "\n";
        file << "      foto: \"" << a.foto << "\"\n";
        file << "      latitud: " << a.latitud << "\n";
        file << "      longitud: " << a.longitud << "\n";
        file << "      descriptores:\n";
        for (const auto& line : a.descriptores_raw) {
            file << line << "\n";
        }
    }

    file.close();

    // Ahora limpiamos el archivo de guiones duplicados
    std::ifstream in("hilera.db");
    std::ofstream out("hilera_temp.db");

    std::string prev_line, curr_line;

    if (std::getline(in, prev_line)) {
        out << prev_line << "\n";
    }

    while (std::getline(in, curr_line)) {
        if (!(curr_line == "   -" && prev_line == "   -")) {
            out << curr_line << "\n";
        }
        prev_line = curr_line;
    }

    in.close();
    out.close();

    // Reemplazar el archivo original
    std::remove("hilera.db");
    std::rename("hilera_temp.db", "hilera.db");

}



int main() {
	db_load();

    // Abrir el archivo listado.txt para leer los nombres de las imágenes
    std::ifstream listado("listado.txt");
    if (!listado.is_open()) {
        std::cerr << "No se pudo abrir el archivo listado.txt" << std::endl;
        return -1;
    }

    // Leer la primera línea que contiene el número de imágenes
    int num_images;
    listado >> num_images;

    // Vector para almacenar los nombres de los archivos de imagen
    std::vector<std::string> image_files(num_images);

    // Leer el resto de las líneas con los nombres de los archivos jpg
    for (int i = 0; i < num_images; ++i) {
        listado >> image_files[i];
    }

    listado.close();

    // Archivo para guardar el ground_truth
    std::ofstream ground_truth("ground_truth.txt", std::ios::app); // Usamos append para no sobreescribir

    // Iterar a través de las imágenes
    for (const auto& image_file : image_files) {
        // Cargar la imagen
        cv::Mat img = cv::imread(image_file);

        if (img.empty()) {
            std::cerr << "No se pudo abrir la imagen: " << image_file << std::endl;
            continue;
        }

	if (db_check(image_file)) {

		cv::putText(img,                          // imagen destino
            "Ya esta en la base",         // texto
            cv::Point(30, 50),            // posición (x, y)
            cv::FONT_HERSHEY_SIMPLEX,     // fuente
            1.0,                          // escala (tamaño)
            cv::Scalar(0, 0, 255),        // color (B, G, R) → rojo
            2,                            // grosor del trazo
            cv::LINE_AA);                 // tipo de línea

	} else {
    // Mostrar cartel: "Nueva foto, se puede guardar con E"
	}

        // Mostrar la imagen
        cv::imshow("Imagen", img);

        // Esperar a que se presione una tecla
        char key = (char)cv::waitKey(0);

        // Si se presiona 'E' o 'e', guardar el nombre del archivo en ground_truth.txt
        if (key == 'E' || key == 'e') {
            ground_truth << image_file << std::endl;
            std::cout << "Guardado en ground_truth: " << image_file << std::endl;
	    db_add(image_file);
        }
    }

    db_save();
    // Cerrar los archivos y ventanas
    ground_truth.close();
    cv::destroyAllWindows();

    return 0;
}

