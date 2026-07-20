
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
    double dt_sec);


