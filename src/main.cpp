
#include <opencv2/opencv.hpp>
#include <iostream>

int main() {
    // Cargar una imagen
    cv::Mat image = cv::imread("imagen.jpg");

    // Verificar si la imagen se cargó correctamente
    if (image.empty()) {
        std::cerr << "Error al cargar la imagen" << std::endl;
        return -1;
    }

    // Mostrar la imagen en una ventana
    cv::imshow("Imagen", image);
    cv::waitKey(0); // Espera hasta que se presione una tecla

    return 0;
}

