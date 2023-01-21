#include <stdio.h>
#include <math.h>
#include "frame.hpp"
#include "point.hpp"

using namespace motion_parallax;

int main() {
    // constructor:
    // camera_bearing, x, z, double[] relative_bearings, Frame prev
    Frame f0(0.0, 0.0, 0.0, {(M_PI / 4.0)}, nullptr);
    Frame f1(0.0, 0.0, 100.0, {0.0}, &f0);

    f1.correlate_to_prev();
    printf("correlated\n");
    f1.triangulate_all_objs();
    printf("triangulated\n");

    std::optional<Point> op = f1.detections().at(0).centroid;

    if (op) {
        printf("x: %f z: %f\n", op.value().x, op.value().y);
    } else {
        printf("this shit don exist\n");
    }
}
