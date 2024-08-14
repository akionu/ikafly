#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#define M (0.1f)
#define G (9.8f)
#define AIR (1.135f)
#define FIELD (30.0f)

int main(int argc, char* argv[]) {
    double v_inf = atof(argv[1]);
    double v_wind_base = atof(argv[2]); // m/s
    double s = atof(argv[3]);
    
    printf("v_wind[m/s],max_height[m]\n");
    for (double i = 0; i <= 20; i+=0.25) {
        double v_wind = v_wind_base + (double)i;
        double F = 0.5 * (AIR) * (v_wind*v_wind) * (s*s);
        double t = sqrt(FIELD * M / F);
        double v = FIELD / t;
        double x = ((v_inf*v_inf)/G)*log(cosh((G*t)/v_inf));
//        printf("in: v_inf: %f [m/s], v_wind: %f [m/s], s: %f [m^2]\n",
//               v_inf, v_wind, s);
//        printf("out: F: %f [N], t: %f [s], v: %f [m/s], x: %f [m]\n", 
//               F, t, v, x);
        printf("%.2f,%.2f\n", v_wind, x);
    }

}
