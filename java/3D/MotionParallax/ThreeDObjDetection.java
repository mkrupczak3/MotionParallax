package MotionParallax;

import MotionParallax.ThreeDFrame;


public class ThreeDObjDetection {

    public ThreeDObjDetection next;
    public ThreeDObjDetection prev;

    public ThreeDPoint centroid = null;

    public ThreeDFrame parent;

    protected ThreeDAngle det_angle; // bearing from vehicle to landmark
    protected ThreeDAngle reverse_angle; // bearing from landmark to the vehicle

    ThreeDObjDetection(ThreeDFrame parent, ThreeDAngle det_angle, ThreeDObjDetection prev) {
        this.parent = parent;
        this.det_angle = det_angle;
        this.reverse_angle = Utils.getReverseAngle(det_angle);
        this.prev = prev;
    }

    public ThreeDAngle getReverseAngle() {
        return reverse_angle;
    }

    public ThreeDAngle getDetAngle() {
        // reverse again to obtain original bearing
        return det_angle;
    }

    public ThreeDFrame getParent() {
        return parent;
    }
}
