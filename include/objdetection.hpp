#pragma once

#include <optional>
#include "point.hpp"
#include "utils.hpp"

namespace motion_parallax {

class Frame; // declaration to avoid circular include

class ObjDetection {
private:
    Frame* parent_;

    double detected_bearing_;
    double reverse_bearing_; // bearing from landmark to the vehicle

public:
    ObjDetection* next;
    ObjDetection* prev;

    std::optional<Point> centroid;

    ObjDetection(Frame* parent, double detected_bearing, ObjDetection* prev);

    double reverse_bearing();
    
    double detected_bearing();
    
    Frame* parent_ptr();
};

}
