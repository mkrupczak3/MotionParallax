#include "objdetection.hpp"

namespace motion_parallax {


ObjDetection::ObjDetection(Frame* parent, double detected_bearing, ObjDetection* prev)
    : parent_(parent), detected_bearing_(detected_bearing), prev(prev) {
    reverse_bearing_ = utils::get_reverse_bearing(detected_bearing);
}

double ObjDetection::reverse_bearing() {
    return reverse_bearing_;
}

double ObjDetection::detected_bearing() {
    return detected_bearing_;
}

Frame* ObjDetection::parent_ptr() {
    return parent_;
}

}
