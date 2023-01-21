#pragma once

#include <stdint.h>
#include <math.h>
#include <vector>
#include "objdetection.hpp"
#include "point.hpp"
#include "utils.hpp"


namespace motion_parallax {

class Frame {
private:
    static uint64_t next_frame_number;



    //static constexpr double CORRELATION_ANGLE_THRESHOLD = M_PI / 8.0; // correct thres
    static constexpr double CORRELATION_ANGLE_THRESHOLD = 2.0 * M_PI; //debug only

    static constexpr double CONE_REVERSE_ANGLE_THRESHOLD = M_PI / 16.0; // correct thres
    //static constexpr double CORRELATION_ANGLE_THRESHOLD = 0.0 * M_PI; //debug only

    static constexpr uint32_t MAX_LIST_LENGTH = 80;

    Point camera_global_pos_;
    std::vector<ObjDetection> detections_;
    uint64_t frame_number_;

public:
    Frame* prev;

    Frame(double camera_bearing, double x, double y, std::vector<double> relative_bearings, Frame* prev_frame);

    ~Frame();

    double get_x() const;

    double get_y() const;

    std::vector<ObjDetection>& detections();

    double bearingtoframe(const Frame& other);

    double disttoframe(const Frame& other);

    // correlate landmarks in cur frame to those in previous frame
    //     where a correlation can be made with certainty, add detection object as head to a linked list
    //     containing each detection of a specific landmark over time
    void correlate_to_prev();

    void triangulate_all_objs();

    void do_triangulation(ObjDetection* head);

    Point center_of(std::vector<Point>& points);
};

}
