package MotionParallax;

import java.util.ArrayList;
import java.util.HashMap;

class Frame {

    protected static long _next_frame_number = 0;

    protected double camera_bearing;
    protected double x;
    protected double z;

    protected double [] detection_bearings;
    protected Hashmap<Double, ObjDetection> detection_objects = new HashMap<>();
    public Frame next;
    public Frame prev;
    public long frame_number;

    public final THRESHOLD1 = Math.PI / 4.0d;
    public final THRESHOLD2 = Math.PI / 8.0d;

    Frame(double camera_bearing, double x, double z, double[] detection_bearings, Frame prev) {
        frame_number = _next_frame_number;
        _next_frame_number++;
        this.camera_bearing = camera_bearing;
        this.x = x;
        this.z = z;
        this.detection_bearings = detection_bearings;
        this.prev = prev;
        genDetectionObjects();
    }

    protected void genDetectionObjects() {
        for (int i = 0; i < detection_bearings.length; i++) {
            detection_objects.put(detection_bearings[i], new ObjDetection(this, detection_bearings[i], null));
        }
        // out_detection_objects = new ObjDetection[detection_bearings.length];
        // for (int i = 0; i < detection_bearings.length; i++) {
        //     out_detection_objects[i] = new ObjDetection(this, detection_bearings[i], null);
        // }
        // detection_objects = out_detection_objects;
    }

    public double x() {
        return x;
    }

    public double z() {
        return z;
    }

    public double getCameraBearing() {
        return camera_bearing;
    }

    public double[] getDetectionBearings() {
        return detection_bearings;
    }

    public Hashmap<Double, ObjDetection> getDetectionObjects() {
        return detection_objects;
    }

    public correlateToPrev() {
        ArrayList<double> usableBearings = new ArrayList<double>(detection_bearings);
        double[] prevBearings = prev.getDetectionBearings();
        double max_diff = 0.0d;
        double diff;
        while (usableBearings.size() > prevBearings.length) {
            max_diff = 0.0d;
            double max_diff_bearing;
            for (double aBearing : usableBearings) {
                for (double aPrevBearing : prevBearings) {
                    diff = Utils.getAngleDiff(aBearing, aPrevBearing);
                    if (diff >= max_diff) {
                        max_diff = diff;
                        max_diff_bearing = aBearing
                    }
                }
            }
            usableBearings.remove(max_diff_bearing);
        }
        while (max_diff >= THRESHOLD1) {
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
        int numRows = prevBearings.length();
        int numCols = usableBearings.size();
        int[][] costMatrix = new int[numRows][numCols];

        double assignee;
        double assignment;
        for (int i = 0; i < numRows; i++) {
            for (int j = 0; j < numCols; j++) {
                assignment = prevBearings[i];
                assignee = usableBearings[j];
                costMatrix[i][j] = (int) (Utils.getAngleDiff(assignee, assignment) / (2 * Math.PI)) * (Integer.MAX_VALUE / 2);
            }
        }
        HungarianAlgorithm ha = new HungarianAlgorithm(costMatrix);
        int[][] assignments = ha.findOptimalAssignment();
        for (int[] anAssignment : assignments) {
            assignee = usableBearings[anAssigment[0]];
            assignment = prevBearings[anAssignment[1]];
            ObjDetection curDetection = detection_objects.get(assignee);
            ObjDetection prevDetection = this.prev.getDetectionObjects().get(assignment);
            curDetection.prev = prevDetection;
            prevDetection.next = curDetection;
        }
    }

}
