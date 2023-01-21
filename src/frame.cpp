#include "frame.hpp"

#include "hungarian.hpp"
#include "stdio.h"


namespace motion_parallax {

uint64_t Frame::next_frame_number = 0;

    Frame::Frame(double camera_bearing, double x, double y, std::vector<double> relative_bearings, Frame* prev)
    : camera_global_pos_(x, y), detections_(), prev(prev), frame_number_() {
    frame_number_ = next_frame_number;
    next_frame_number++;

    if (frame_number_ >= MAX_LIST_LENGTH) {
        Frame* cur = this;
        while (cur->prev->prev != nullptr) {
            cur = cur->prev;
        }

        Frame* last = cur->prev;
        cur->prev = nullptr;
        delete last;
    }

    camera_global_pos_ = Point(x, y);
    for (int i = 0; i < relative_bearings.size(); i++) {
        double d = relative_bearings[i];

        // convert to absolute bearings
        detections_.emplace_back(this, utils::normalized(camera_bearing + d), nullptr);
    }
}

Frame::~Frame() {
    for (auto& det : detections_) {
        if (det.next != nullptr) {
            det.next->prev = nullptr;
        }
    }
}

double Frame::get_x() const {
    return camera_global_pos_.x;
}

double Frame::get_y() const {
    return camera_global_pos_.y;
}

std::vector<ObjDetection>& Frame::detections() {
    return detections_;
}

double Frame::bearingtoframe(const Frame& other) {
    double deltaz = other.get_y() - get_y();
    double deltax = other.get_x() - get_x();
    return atan2(deltaz, deltax);
}

double Frame::disttoframe(const Frame& other) {
    double deltaz = other.get_y() - get_y();
    double deltax = other.get_x() - get_x();
    return sqrt(deltaz*deltaz + deltax*deltax);
}

// correlate landmarks in cur frame to those in previous frame
//     where a correlation can be made with certainty, add detection object as head to a linked list
//     containing each detection of a specific landmark over time
void Frame::correlate_to_prev() {
    std::vector<ObjDetection>& prev_detections = prev->detections();
    if (prev_detections.empty()) {
        return;
    }

    std::vector<ObjDetection*> usable_detections;
    for (auto& det : detections_) {
        usable_detections.push_back(&det);
    }

    double max_min_diff = 0.0;
    std::vector<ObjDetection*>::iterator max_min_diff_it;
    double min_diff = 2 * M_PI;
    std::vector<ObjDetection*>::iterator min_diff_it;
    double diff;
    while (usable_detections.size() > prev_detections.size()) { // cull current landmark detections
        for (auto uit = usable_detections.begin(); uit != usable_detections.end(); uit++) {
            min_diff = 2 * M_PI;
            double usable_det_bearing = (*uit)->detected_bearing();
            for (auto pit = prev_detections.begin(); pit != prev_detections.end(); pit++) {
                double prev_det_bearing = pit->detected_bearing();
                diff = utils::get_angle_diff(usable_det_bearing, prev_det_bearing);
                if (diff <= min_diff) {
                    min_diff = diff;
                    min_diff_it = uit;
                }
            }
            if (min_diff >= max_min_diff) {
                max_min_diff = min_diff;
                max_min_diff_it = min_diff_it;
            }
        }

        usable_detections.erase(max_min_diff_it);
    }

    if (usable_detections.empty()) {
        return;
    }

    // cull landmark detections too different from any prev detections
    while (max_min_diff >= CORRELATION_ANGLE_THRESHOLD) {
        for (auto uit = usable_detections.begin(); uit != usable_detections.end(); uit++) {
            min_diff = 2 * M_PI;
            double usable_det_bearing = (*uit)->detected_bearing();
            for (auto pit = prev_detections.begin(); pit != prev_detections.end(); pit++) {
                double prev_det_bearing = pit->detected_bearing();
                diff = utils::get_angle_diff(usable_det_bearing, prev_det_bearing);
                if (diff <= min_diff) {
                    min_diff = diff;
                    min_diff_it = uit;
                }
            }
            if (min_diff >= max_min_diff) {
                max_min_diff = min_diff;
                max_min_diff_it = min_diff_it;
            }
        }

        if (max_min_diff >= CORRELATION_ANGLE_THRESHOLD) {
            usable_detections.erase(max_min_diff_it);
        }
    }

    if (usable_detections.empty()) {
        return;
    }

    int num_rows = usable_detections.size();
    int num_cols = prev_detections.size();
    std::vector<std::vector<double>> cost_matrix;

    for (ObjDetection* assignee : usable_detections) {
        auto& row = cost_matrix.emplace_back();
        for (ObjDetection& assignment : prev_detections) {
            row.push_back(utils::get_angle_diff(assignee->detected_bearing(),
                                                assignment.detected_bearing()));
        }
    }

    std::vector<int> assignments;


    //printf("[\n");
    for (auto row : cost_matrix) {
        //printf("[ ");
        for (auto el : row) {
            //printf("%f ", el);
        }
        //printf(" ]\n");
    }
    //printf("]\n");


    HungarianAlgorithm ha;
    ha.Solve(cost_matrix, assignments);

    //printf("[ ");
    for (auto n : assignments) {
        //printf("%d ", n);
    }
    //printf("]\n");

    // find assignment for only non-placeholder columns
    for (int i = 0; i < usable_detections.size(); i++) {
        ObjDetection* assignee = usable_detections[i];
        ObjDetection& assignment = prev_detections[assignments[i]];

        assignee->prev = &assignment;
        assignment.next = assignee;
    }
}



void Frame::triangulate_all_objs() {
    for (auto& det : detections_) {
        do_triangulation(&det);
    }
}

void Frame::do_triangulation(ObjDetection* head) {


    if (head == nullptr || head->prev == nullptr) {
        return;
    }

    ObjDetection* cur = head->prev;
    std::vector<ObjDetection*> usable_precursors;
    int length_at_cur = 1;
    do {
        if (length_at_cur < MAX_LIST_LENGTH) {
            if (utils::get_angle_diff(head->reverse_bearing(),
                                    cur->reverse_bearing())
                >= CONE_REVERSE_ANGLE_THRESHOLD) {
                usable_precursors.push_back(cur);
            }
        } else {
            cur->prev = nullptr;
        }
        cur = cur->prev;
        length_at_cur++;
    } while (cur != nullptr);

    //printf("got here\n");

    if (usable_precursors.empty()) {
        return;
    }

    double c; // meters
    double B; // radians
    double C; // radians

    double b; // meters

    std::vector<Point> triangulations;
    double x;
    double y;

    for (ObjDetection* det : usable_precursors) {
        B = utils::get_angle_diff(det->parent_ptr()->bearingtoframe(*(head->parent_ptr())),
                                det->detected_bearing());
        C = utils::get_angle_diff(det->reverse_bearing(),
                                head->reverse_bearing());
        c = det->parent_ptr()->disttoframe(*(head->parent_ptr()));

        b = c * sin(B) / sin(C);

        x = get_x() + b * cos(head->detected_bearing());
        y = get_y() + b * sin(head->detected_bearing());
        triangulations.emplace_back(x, y);
    }

    // write estimated location back to ObjDetection in this frame
    //printf("got here\n");
    head->centroid = center_of(triangulations);
}

Point Frame::center_of(std::vector<Point>& points) {
    double xavg = 0.0;
    double yavg = 0.0;
    for (Point& point : points) {
        xavg += point.x;
        yavg += point.y;
    }
    xavg /= points.size();
    yavg /= points.size();
    return Point(xavg, yavg);
}

}
