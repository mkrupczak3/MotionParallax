package MotionParallax;

import MotionParallax.Frame;

class ObjDetection {

    public ObjDetection next;
    public ObjDetection prev;

    Double cartesianX;
    Double cartesianZ;

    Frame parent;

    protected double reverse_bearing; // bearing from landmark to the vehicle

    ObjDetection(Frame parent, double bearing, ObjDetection prev) {
        this.parent = parent;
        this.reverse_bearing = Utils.getReverseBearing(bearing);
        this.prev = prev;
    }
}
