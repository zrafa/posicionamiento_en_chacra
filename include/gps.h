/*
 * gps.h: procesa las tramas del gps
 * dentro de las hileras de una chacra.
 */


// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

#ifndef GPS_H
#define GPS_H


// Estructura para almacenar los datos GPS
struct GPS_data {
    long long timestamp_us;
    double latitude;
    double longitude;
};

void gps_get_lat_lon(long long tiempo_us, double *latitud, double *longitud);

#endif  // GPS_H
