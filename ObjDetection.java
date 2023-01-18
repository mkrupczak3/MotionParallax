package MotionParallax;

import MotionParallax.Frame;


class ObjDetection {

    public ObjDetection next;
    public ObjDetection prev;

    public Point centroid = null;

    Frame parent;

    protected double reverse_bearing; // bearing from landmark to the vehicle

    ObjDetection(Frame parent, double bearing, ObjDetection prev) {
        this.parent = parent;
        this.reverse_bearing = Utils.getReverseBearing(bearing);
        this.prev = prev;
    }

    public double get_reverse_bearing() {
        return reverse_bearing;
    }

    public double get_detected_bearing() {
        // reverse again to obtain original bearing
        return Utils.getReverseBearing(reverse_bearing);
    }

    public Frame getParent() {
        return parent;
    }
}
