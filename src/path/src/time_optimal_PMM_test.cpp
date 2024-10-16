#include "time_optimal_PMM.h"

int main() {
    // Initial conditions for x, y, z axes
    Vector3 x0, v0, xT, vT, umax, umin, vmax, vmin;
    // x0.x = 0.; x0.y = 0.; x0.z = 0.;
    // v0.x = 0.; v0.y = 0.; v0.z = 0.;
    // xT.x = -8.; xT.y = 7.; xT.z = 2.;
    // vT.x = 3.; vT.y = 6.; vT.z = 0.;
    // umax.x = 12.; umax.y = 12.; umax.z = 2.5;
    // umin.x = 12.; umin.y = 12.; umin.z = 15.;
    // vmax.x = 7.5; vmax.y = 7.5; vmax.z = 5.;
    // vmin.x = 7.5; vmin.y = 7.5; vmin.z = 5.;
    // x0.x = -6.39; x0.y = 9.92; x0.z = 21.56;
    // v0.x = -4.8; v0.y = 1.31; v0.z = 0.87;
    // xT.x = -5; xT.y = 10; xT.z = 20.24;
    // vT.x = 0.; vT.y = 0.; vT.z = 1.;
    // umax.x = 12.; umax.y = 12.; umax.z = 3.;
    // umin.x = 12.; umin.y = 12.; umin.z = 7.;
    // vmax.x = 10.; vmax.y = 10.; vmax.z = 5.;
    // vmin.x = 10.; vmin.y = 10.; vmin.z = 5.;
    // x0.x = 0.; x0.y = 0.; x0.z = 0.;
    // v0.x = 0.; v0.y = 0.; v0.z = 0.;
    // xT.x = 2.; xT.y = 5.; xT.z = 9.2;
    // vT.x = 0.; vT.y = 0.; vT.z = 0.8;
    // umax.x = 12.; umax.y = 12.; umax.z = 3.;
    // umin.x = 12.; umin.y = 12.; umin.z = 7.;
    // vmax.x = 10.; vmax.y = 10.; vmax.z = 5.;
    // vmin.x = 10.; vmin.y = 10.; vmin.z = 5.;
    x0.x = 0.; x0.y = -0.02; x0.z = 5.49;
    v0.x = 0.076; v0.y = -0.05; v0.z = 7.17;
    xT.x = 2; xT.y = 5; xT.z = 15.43;
    vT.x = 0.; vT.y = 0.; vT.z = 0.8;
    umax.x = 12.; umax.y = 12.; umax.z = 3.;
    umin.x = 12.; umin.y = 12.; umin.z = 7.;
    vmax.x = 10.; vmax.y = 10.; vmax.z = 5.;
    vmin.x = 10.; vmin.y = 10.; vmin.z = 5.;
    TimeOptimalPMM3D pmm3d(x0, v0, xT, vT, umax, umin, vmax, vmin);

    pmm3d.compute_times();
    pmm3d.plot_trajectory();

    return 0;
}
