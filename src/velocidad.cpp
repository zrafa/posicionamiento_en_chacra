
#include <opencv2/opencv.hpp>
#include <iostream>
#include <thread>
#include <semaphore>

#include <db.h>
#include <gps.h>
#include <ui_gfx.h>


extern frutal ultimos_arboles[];
extern int total;

float pixel_to_meters(float pixel_displacement, float Z_meters);

std::binary_semaphore sem(0); // empieza en 0 → bloqueado
std::binary_semaphore sem_continue(0); // empieza en 0 → bloqueado

double vel_px_n = 0;
double vel_px_total = 0;

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
		else {
			vel_px_total += vel_seg;
			vel_px_n++;
		}

		// gui
		ostringstream texto;
		texto << "vel: " << vel_seg << " m/s    ";
		mostrar_texto(ventana_completa, texto, 1100, 900);

		double vel_gps = gps_get_speed(ts1+((ts2-ts1)/2));

                std::cout << "VELOCIDAD: Desplazamiento horizontal calculado: "
                        << dx << " píxeles " << ultimos_arboles[0].foto << " " <<
                        ultimos_arboles[total-1].foto << " " << vel_seg << " " << (vel_px_total/vel_px_n) << "prom " << distancia_prom << " " << ultimos_arboles[0].center_x << " " << ultimos_arboles[total-1].center_x << " " << vel_gps << " " << ((vel_seg+vel_gps/100.0)/2.0) << std::endl;

                sem_continue.release();
        }
}


/* como adaptamos fx:
 *
 *  // Parámetro de calibración original (640x480)
 *   const float fx_original = 820.2028f;
 *   const float width_original = 640.0f;
 *
 *  // Resolución real actual (la que te devuelve la cámara)
 *  const float width_current = 864.0f;
 *
 *  // Escalado de la focal
 *  float scale_x = width_current / width_original;
 *  float fx = fx_original * scale_x;
 */

float pixel_to_meters(float pixel_displacement, float Z_meters)
{
    // const float fx = 720.0f; // AJUSTADO EMPÍRICAMENTE
    const float fx = 820.2f; // ORIGINAL 640px, fx=820.2 
    				// segun la teoria esa camara c525 tiene un 
				// sensor que puede de modo crudo un ancho a
				// 864px. Así que fx no cambia en principio.
				// (640x480 simplemente recorta los pixeles de
				// los costados)

    return (pixel_displacement * Z_meters) / fx;
}



