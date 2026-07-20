
#include <math.h>

#define DEG_TO_RAD (M_PI / 180.0)

typedef struct
{
    double x;            // metros
    double y;            // metros
    double heading_deg;  // grados
} Pose2D;


/*
    x,y anterior
    heading_prev_deg -> heading anterior
    heading_curr_deg -> heading actual
    speed_cms        -> velocidad actual en cm/s
    dt_sec           -> delta t en segundos

    retorna pose nueva
*/
Pose2D update_pose(
    double x,
    double y,
    double heading_prev_deg,
    double heading_curr_deg,
    double speed_cms,
    double dt_sec)
{
    Pose2D p;

    // cm/s -> m/s
    double speed = speed_cms / 100.0;

    // promedio angular correcto
    double delta =
        fmod((heading_curr_deg - heading_prev_deg + 540.0), 360.0)
        - 180.0;

    double heading_avg_deg =
        heading_prev_deg + delta * 0.5;

    // GPS/compass convention:
    // 0° = Norte
    // 90° = Este
    //
    // Convertimos a sistema matemático:
    // 0 rad = +X
    // pi/2 = +Y

    double theta =
        (90.0 - heading_avg_deg) * DEG_TO_RAD;

    // distancia recorrida
    double distance = speed * dt_sec;

    // integración
    double dx = distance * cos(theta);
    double dy = distance * sin(theta);

    p.x = x + dx;
    p.y = y + dy;

    p.heading_deg = heading_curr_deg;

    return p;
}
