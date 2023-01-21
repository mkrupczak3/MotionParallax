#include "utils.hpp";

#include <math.h>

namespace motion_parallax {
namespace utils {

double normalized(double inAngle) {
    double outAngle = inAngle;
    while (outAngle >= 2.0d * M_PI) {
        outAngle -= 2.0d * M_PI;
    }
    while (outAngle < 0.0d) {
        outAngle += 2.0d * M_PI;
    }
    return outAngle;
}

double get_reverse_bearing(double inAngle) {
    return normalized(inAngle + M_PI);
}

double get_angle_diff(double A, double B) {
    double diff = normalized(A - B);
    if (diff > M_PI) { // if angle is a reflex angle, get its smaller equivalent
        diff = 2.0d * M_PI - diff;
    }
    return diff;
}

}
}
