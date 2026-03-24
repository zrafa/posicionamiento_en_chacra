
#include <opencv2/opencv.hpp>
#include <iostream>
#include <thread>
#include <semaphore>

#include <db.h>

extern frutal ultimos_arboles[];
extern int total;

float pixel_to_meters(float pixel_displacement, float Z_meters);

std::binary_semaphore sem(0); // empieza en 0 → bloqueado
std::binary_semaphore sem_continue(0); // empieza en 0 → bloqueado

void velocidad() {

        long long ts1, ts2;
        float distancia_prom = 0;
        float vel_seg = 0;
        while (1) {
                sem.acquire();

                ts1 = ultimos_arboles[0].ts;
                ts2 = ultimos_arboles[total-1].ts;

                for (int i=0; i<total-1; i++)
                        distancia_prom += ultimos_arboles[i].distancia;
                distancia_prom /= total;

                // original: vel_seg = 1000000.0 * pixel_to_meters(dx, distancia_prom) / (ts2-ts1);
                //
                double dt = (ts2 - ts1) / 1e6; // segundos
                double Z = distancia_prom / 100.0; // cm a metros

                // dx: movimiento en pixeles, usando solo centerX
                float dx = ultimos_arboles[total-1].center_x - ultimos_arboles[0].center_x;
                double dx_m = pixel_to_meters(dx, Z);

                double vel_seg = dx_m / dt;
                //vel_seg = 1000000.0 * pixel_to_meters(dx, distancia_prom/100.0) / (ts2-ts1);

		// valores muy bajos son outliers. Esto tiene que ser calibrado cuando se
		// sepa la verdadera velocidad constante de ensayos
		if (vel_seg < 0.2)
			vel_seg = -1;

                std::cout << "VELOCIDAD: Desplazamiento horizontal calculado: "
                        << dx << " píxeles " << ultimos_arboles[0].foto << " " <<
                        ultimos_arboles[total-1].foto << " " << vel_seg << " " << ts2-ts1 << " " << distancia_prom << " " << ultimos_arboles[0].center_x << " " << ultimos_arboles[total-1].center_x << std::endl;

                sem_continue.release();
        }
}



float pixel_to_meters(float pixel_displacement, float Z_meters)
{
    const float fx = 620.0f; // AJUSTADO EMPÍRICAMENTE

    return (pixel_displacement * Z_meters) / fx;
}



