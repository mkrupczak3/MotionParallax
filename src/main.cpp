#include <stdio.h>
#include <math.h>
#include "frame.hpp"
#include "point.hpp"

using namespace motion_parallax;

int main() {
    // constructor:
    // camera_bearing, x, z, double[] relative_bearings, Frame prev


    Frame f0(0.0, 0.0, 0.0, { (M_PI / 4.0), ((3 * M_PI) / 4) }, nullptr);
    Frame f1(0.0, 0.0, 100.0, { M_PI, 0.0 }, &f0);

    f1.correlate_to_prev();
    printf("correlated\n");
    f1.triangulate_all_objs();
    printf("triangulated\n");

    for (auto det : f1.detections()) {
        printf("=======\n");
        std::optional<Point> op = det.centroid;

        if (op) {
            printf("x: %f y: %f\n", op.value().x, op.value().y);
        } else {
            printf("this shit don exist\n");
        }
        printf("=======\n");
    }
}
