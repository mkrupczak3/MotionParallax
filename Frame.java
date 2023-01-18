package MotionParallax;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;
import java.util.Map;
import java.util.Set;

import MotionParallax.Utils;
import MotionParallax.ObjDetection;
import MotionParallax.Point;

class Frame {

    protected static long _next_frame_number = 0;

    protected double camera_bearing;

    protected Point camera_point;

    protected double [] detection_bearings;
    protected HashMap<Double, ObjDetection> detection_objects = new HashMap<>();

    public Frame prev;
    public long frame_number;

    public final double THRESHOLD1 = Math.PI / 8.0d;
    public final double THRESHOLD2 = Math.PI / 16.0d;
    public final int MAXOBJLISTLENGTH = 80;

    Frame(double camera_bearing, double x, double z, double[] relative_bearings, Frame prev) {
        frame_number = _next_frame_number;
        _next_frame_number++;
        this.camera_bearing = camera_bearing;
        this.camera_point = new Point(x, z);
        double[] absolute_bearings = new double[relative_bearings.length];
        for (int i = 0; i < absolute_bearings.length; i++) {
            double d = relative_bearings[i];
            absolute_bearings[i] = Utils.normalized(camera_bearing + d);
        }
        this.detection_bearings = absolute_bearings;
        this.prev = prev;
        genDetectionObjects();
    }

    protected void genDetectionObjects() {
        for (int i = 0; i < detection_bearings.length; i++) {
            detection_objects.put(detection_bearings[i], new ObjDetection(this, detection_bearings[i], null));
        }
    }

    public double x() {
        return camera_point.x();
    }

    public double z() {
        return camera_point.z();
    }

    public double getCameraBearing() {
        return camera_bearing;
    }

    public double[] getDetectionBearings() {
        return detection_bearings;
    }

    public HashMap<Double, ObjDetection> getDetectionObjects() {
        return detection_objects;
    }

    public double bearingToFrame(Frame other) {
        double deltaz = other.z() - z();
        double deltax = other.x() - x();
        return Math.atan2(deltaz, deltax);
    }

    public double distToFrame(Frame other) {
        double deltaz = other.z() - z();
        double deltax = other.x() - x();
        return Math.sqrt(deltaz*deltaz + deltax*deltax);
    }

    public void clearChildren() {
        for (double aBearing : getDetectionBearings()) {
            anObj = detection_objects[aBearing];
            if (anObj != null) {
                anObj.prev = null;
                anObj.next = null;
                anObj.parent = null;
                detection_objects[aBearing] = null;
            }
        }
        if (this.prev != null) {
            this.prev.clearChildren()
        }
        this.prev = null;
    }

    // Correlate landmarks in cur frame to those in previous frame
    //     Where a correlation can be made with certainty, add detection object as head to a linked list
    //     containing each detection of a specific landmark over time
    public void correlateToPrev() {
        ArrayList<Double> usableBearings = new ArrayList<Double>(detection_bearings.length);
        for (double d : detection_bearings) usableBearings.add(d);

        double[] prevBearings = prev.getDetectionBearings();
        double max_diff = 0.0d;
        double max_diff_bearing = 99999.0d; //value never used
        double diff;
        while (usableBearings.size() > prevBearings.length) { // cull current landmark detections
            max_diff = 0.0d;
            for (double aBearing : usableBearings) {
                for (double aPrevBearing : prevBearings) {
                    diff = Utils.getAngleDiff(aBearing, aPrevBearing);
                    if (diff >= max_diff) {
                        max_diff = diff;
                        max_diff_bearing = aBearing;
                    }
                }
            }
            usableBearings.remove(max_diff_bearing);
        }
        while (max_diff >= THRESHOLD1) { // cull landmark detections too far from any prev detections
            max_diff = 0.0d;
            for (double aBearing : usableBearings) {
                for (double aPrevBearing : prevBearings) {
                    diff = Utils.getAngleDiff(aBearing, aPrevBearing);
                    if (diff >= max_diff) {
                        max_diff = diff;
                        max_diff_bearing = aBearing;
                    }
                }
            }
            if (max_diff >= THRESHOLD1) {
                usableBearings.remove(max_diff_bearing);
            }
        }
        int numRows = prevBearings.length;
        int numCols = usableBearings.size();
        int[][] costMatrix = new int[numRows][numCols];

        double assignee;
        double assignment;
        for (int i = 0; i < numRows; i++) {
            for (int j = 0; j < numCols; j++) {
                assignment = prevBearings[i];
                assignee = usableBearings.get(j);
                costMatrix[i][j] = (int) (Utils.getAngleDiff(assignee, assignment) / (2 * Math.PI)) * (Integer.MAX_VALUE / 2); // convert angle diff to an integer score
            }
        }
        HungarianAlgorithm ha = new HungarianAlgorithm(costMatrix);
        int[][] assignments = ha.findOptimalAssignment();
        for (int[] anAssignment : assignments) {
            assignee = usableBearings.get(anAssignment[0]);
            assignment = prevBearings[anAssignment[1]];
            ObjDetection curDetection = detection_objects.get(assignee);
            ObjDetection prevDetection = this.prev.getDetectionObjects().get(assignment);
            curDetection.prev = prevDetection;
            prevDetection.next = curDetection;
        }
    }



    public void triangulate_all_objs() {
        ArrayList<ObjDetection> allObjs = get_all_objs();

        for (ObjDetection anObj : allObjs) {
            do_triangulation(anObj);
        }
    }

    public ArrayList<ObjDetection> get_all_objs() {
        ArrayList<ObjDetection> allObjs = new ArrayList<ObjDetection>(getDetectionBearings().length);
        Set set = getDetectionObjects().entrySet();
        Iterator i = set.iterator();
        while (i.hasNext()) {
            Map.Entry me = (Map.Entry)i.next();
            allObjs.add((ObjDetection) me.getValue());
        }
        return allObjs;
    }

    public void do_triangulation(ObjDetection head) {
        if (head.prev == null) {
            return;
        }
        ObjDetection cur = head.prev;
        ArrayList<ObjDetection> usablePrecursors = new ArrayList<ObjDetection>();
        int lengthAtCur = 1;
        do {
            if (lengthAtCur < MAXOBJLISTLENGTH) {
                if (Utils.getAngleDiff(head.get_reverse_bearing(), cur.get_reverse_bearing()) >= THRESHOLD2) {
                    usablePrecursors.add(cur);
                }
            } else {
                cur.prev.next = null; // de-alloc all LL nodes and frames beyond max allowed length
                cur.prev = null;
                cur.parent = null;
            }
            cur = cur.prev;
            lengthAtCur++;
        } while(cur != null);

        if (usablePrecursors.size() == 0) {
            return;
        }

        double c; // meters
        double B; // radians
        double C; // radians

        double b; // meters

        ArrayList<Point> triangulations = new ArrayList<Point>(usablePrecursors.size());
        double x;
        double z;

        for (ObjDetection anObj : usablePrecursors) {
            B = Utils.getAngleDiff(anObj.getParent().bearingToFrame(head.getParent()), anObj.get_detected_bearing());
            C = Utils.getAngleDiff(anObj.get_reverse_bearing(), head.get_reverse_bearing());
            c = anObj.getParent().distToFrame(head.getParent());

            b = c * Math.sin(B) / Math.sin(C);

            x = b * Math.cos(head.get_detected_bearing());
            z = b * Math.sin(head.get_detected_bearing());
            triangulations.add(new Point(x, z));
        }

        head.centroid = centerOf(triangulations); // write estimated location back to ObjDetection in this frame
    }

    public Point centerOf(ArrayList<Point> points) {
        double xavg = 0.0d;
        double zavg = 0.0d;
        for (Point aPoint : points) {
            xavg += aPoint.x();
            zavg += aPoint.z();
        }
        xavg /= points.size();
        zavg /= points.size();
        return new Point(xavg, zavg);
    }

}
