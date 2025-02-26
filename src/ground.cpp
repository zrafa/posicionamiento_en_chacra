#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <vector>
#include <string>

int main() {
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

        // Mostrar la imagen
        cv::imshow("Imagen", img);

        // Esperar a que se presione una tecla
        char key = (char)cv::waitKey(0);

        // Si se presiona 'E' o 'e', guardar el nombre del archivo en ground_truth.txt
        if (key == 'E' || key == 'e') {
            ground_truth << image_file << std::endl;
            std::cout << "Guardado en ground_truth: " << image_file << std::endl;
        }
    }

    // Cerrar los archivos y ventanas
    ground_truth.close();
    cv::destroyAllWindows();

    return 0;
}

