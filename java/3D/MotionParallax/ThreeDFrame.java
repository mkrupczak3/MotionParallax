package MotionParallax;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;
import java.util.Map;
import java.util.Set;

import MotionParallax.Utils;
import MotionParallax.ThreeDObjDetection;
import MotionParallax.ThreeDPoint;
import MotionParallax.ThreeDAngle;

public class ThreeDFrame {

    protected static long _next_frame_number = 0;

    protected ThreeDAngle camera_angle;

    protected ThreeDPoint camera_point;

    protected ThreeDAngle [] detection_angles;
    protected HashMap<ThreeDAngle, ThreeDObjDetection> detection_objects = new HashMap<>();

    public ThreeDFrame prev;
    public long frame_number;

    public final double THRESHOLD1 = 2.0d * Math.PI; // debug only
    // public final double THRESHOLD1 = Math.PI / 8.0d; // correct thres

    public final double THRESHOLD2 = 0.0 * Math.PI; // debug only
    // public final double THRESHOLD2 = Math.PI / 16.0d; // correct thres

    public final int MAXLISTLENGTH = 160;

    public ThreeDFrame(ThreeDAngle camera_angle, ThreeDPoint xyz, ThreeDAngle [] relative_angles, ThreeDFrame prev) {
        frame_number = _next_frame_number;
        _next_frame_number++;

        if (frame_number >= MAXLISTLENGTH) {
            ThreeDFrame cur = this;
            while (cur.prev.prev != null) {
                cur = cur.prev;
            }
            cur.prev.clearChildren();
            cur.prev.camera_point = null;
            cur.prev = null;
        }

        this.camera_angle = camera_angle;
        this.camera_point = xyz;
        ThreeDAngle [] absolute_angles = new ThreeDAngle[relative_angles.length];
        for (int i = 0; i < absolute_angles.length; i++) {
            ThreeDAngle rel  = relative_angles[i];
            absolute_angles[i] = camera_angle.sumWith(rel);
        }
        this.detection_angles = absolute_angles;
        this.prev = prev;
        genDetectionObjects();
        correlateToPrev();
    }

    protected void genDetectionObjects() {
        for (int i = 0; i < detection_angles.length; i++) {
            detection_objects.put(detection_angles[i], new ThreeDObjDetection(this, detection_angles[i], null));
        }
    }

    public double x() {
        return camera_point.x();
    }

    public double y() {
        return camera_point.y();
    }

    public double z() {
        return camera_point.z();
    }

    public ThreeDPoint camera_point() {
        return camera_point;
    }

    public ThreeDAngle getCameraAngle() {
        return camera_angle;
    }

    public ThreeDAngle[] getDetectionAngles() {
        return detection_angles;
    }

    public HashMap<ThreeDAngle, ThreeDObjDetection> getDetectionObjects() {
        return detection_objects;
    }

    public ThreeDAngle angleToFrame(ThreeDFrame other) {
        double deltax = other.x() - x();
        double deltay = other.y() - y();
        double deltaz = other.z() - z();
        double xyAngle = Math.atan2(deltay, deltax);
        double xzAngle = Math.atan2(deltaz, deltax);
        return new ThreeDAngle(xyAngle, xzAngle);
    }

    public double distToFrame(ThreeDFrame other) {
        double deltax = other.x() - x();
        double deltay = other.y() - y();
        double deltaz = other.z() - z();
        return Math.sqrt(deltax*deltax + deltay*deltay + deltaz*deltaz);
    }

    public void clearChildren() {
        for (ThreeDAngle anAngle : getDetectionAngles()) {
            ThreeDObjDetection anObj = detection_objects.get(anAngle);
            if (anObj != null) {
                anObj.prev = null;
                if (anObj.next != null) {
                    anObj.next.prev = null;
                }
                anObj.next = null;
                anObj.parent = null;
                detection_objects.put(anAngle, null);
            }
        }
        if (this.prev != null) {
            this.prev.clearChildren();
        }
        this.prev = null;
    }

    // Correlate landmarks in cur frame to those in previous frame
    //     Where a correlation can be made with certainty, add detection object as head to a linked list
    //     containing each detection of a specific landmark over time
    public void correlateToPrev() {
        if (prev == null) {
            return;
        }

        ArrayList<ThreeDAngle> usableAngles = new ArrayList<ThreeDAngle>(detection_angles.length);
        for (ThreeDAngle a : detection_angles) usableAngles.add(a);

        ThreeDAngle[] prevAngles = prev.getDetectionAngles();
        if (prevAngles == null || prevAngles.length == 0) {
            return;
        }

        double max_min_diff = 0.0d;
        ThreeDAngle max_min_diff_angle = null;
        double min_diff = 2 * Math.PI;
        ThreeDAngle min_diff_angle = null;
        double diff;
        while (usableAngles.size() > prevAngles.length) { // cull current landmark detections
            for (ThreeDAngle anAngle : usableAngles) {
                min_diff = 2 * Math.PI;
                for (ThreeDAngle aPrevAngle : prevAngles) {
                    diff = Utils.getAngleDiff(anAngle, aPrevAngle);
                    if (diff <= min_diff) {
                        min_diff = diff;
                        min_diff_angle = anAngle;
                    }
                }
                if (min_diff >= max_min_diff) {
                    max_min_diff = min_diff;
                    max_min_diff_angle = min_diff_angle;
                }
            }
            usableAngles.remove(max_min_diff_angle);
        }

        if (usableAngles.size() == 0) {
            return;
        }

        while (max_min_diff >= THRESHOLD1) { // cull landmark detections too different from any prev detections
            for (ThreeDAngle anAngle : usableAngles) {
                min_diff = 2 * Math.PI;
                for (ThreeDAngle aPrevAngle : prevAngles) {
                    diff = Utils.getAngleDiff(anAngle, aPrevAngle);
                    if (diff <= min_diff) {
                        min_diff = diff;
                        min_diff_angle = anAngle;
                    }
                }
                if (min_diff >= max_min_diff) {
                    max_min_diff = min_diff;
                    max_min_diff_angle = min_diff_angle;
                }
            }
            if (max_min_diff >= THRESHOLD1) {
                usableAngles.remove(max_min_diff_angle);
            }
        }

        if (usableAngles.size() == 0) {
            return;
        }

        int numRows = prevAngles.length;
        int numCols = numRows;
        int[][] costMatrix = new int[numRows][numCols];

        ThreeDAngle assignee;
        ThreeDAngle assignment;
        for (int i = 0; i < numRows; i++) {
            for (int j = 0; j < numCols; j++) {
                if (j < usableAngles.size()) {
                    assignment = prevAngles[i];
                    assignee = usableAngles.get(j);
                    costMatrix[i][j] = (int) (Utils.getAngleDiff(assignee, assignment) / (2 * Math.PI)) * (Integer.MAX_VALUE / 2); // convert angle diff to an integer score
                } else { // to make the cost matrix square, insert placeholder value of max cost
                    costMatrix[i][j] = Integer.MAX_VALUE / 2;
                }
            }
        }
        HungarianAlgorithm ha = new HungarianAlgorithm(costMatrix);
        int[][] assignments = ha.findOptimalAssignment();
        for (int i = 0; i < usableAngles.size(); i++) { // find assignment for only non-placeholder columns
            int[] anAssignment = assignments[i];
            assignee = usableAngles.get(anAssignment[0]);
            assignment = prevAngles[anAssignment[1]];
            ThreeDObjDetection curDetection = detection_objects.get(assignee);
            ThreeDObjDetection prevDetection = this.prev.getDetectionObjects().get(assignment);
            curDetection.prev = prevDetection;
            prevDetection.next = curDetection;
        }
    }



    public void triangulate_all_objs() {
        ArrayList<ThreeDObjDetection> allObjs = get_all_objs();

        for (ThreeDObjDetection anObj : allObjs) {
            do_triangulation(anObj);
        }
    }

    public ArrayList<ThreeDObjDetection> get_all_objs() {
        ArrayList<ThreeDObjDetection> allObjs = new ArrayList<ThreeDObjDetection>(getDetectionAngles().length);
        Set set = getDetectionObjects().entrySet();
        Iterator i = set.iterator();
        while (i.hasNext()) {
            Map.Entry me = (Map.Entry)i.next();
            allObjs.add((ThreeDObjDetection) me.getValue());
        }
        return allObjs;
    }

    public void do_triangulation(ThreeDObjDetection head) {
        if (head.prev == null) {
            return;
        }
        ThreeDObjDetection cur = head.prev;
        ArrayList<ThreeDObjDetection> usablePrecursors = new ArrayList<ThreeDObjDetection>();
        int lengthAtCur = 1;
        do {
            if (lengthAtCur < MAXLISTLENGTH) {
                if (Utils.getAngleDiff(head.getReverseAngle(), cur.getReverseAngle()) >= THRESHOLD2) {
                    usablePrecursors.add(cur);
                }
            } else {
                cur.prev.next = null; // de-alloc all LL nodes beyond max allowed length
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
        double B; // 3DAngle, radians x2
        double C; // radians

        double b; // meters

        ArrayList<ThreeDPoint> triangulations = new ArrayList<ThreeDPoint>(usablePrecursors.size());
        double x;
        double y;
        double z;

        for (ThreeDObjDetection anObj : usablePrecursors) {
            // triangulation is performed on plane defined by cur camera_point, prev camera_point, and object
            B = Utils.getAngleDiff(anObj.getParent().angleToFrame(head.getParent()), anObj.getDetAngle());
            // System.out.printf("B: %f\n", Math.toDegrees(B));
            C = Utils.getAngleDiff(anObj.getReverseAngle(), head.getReverseAngle());
            // System.out.printf("C: %f\n", Math.toDegrees(C));
            c = anObj.getParent().distToFrame(head.getParent());
            // System.out.printf("c: %f\n", c);
            b = c * Math.sin(B) / Math.sin(C); // calculated dist to target from cur camera_point. Triangle does not have to contain a right angle
            // System.out.printf("b: %f\n", b);

            x = x() + b * Math.cos(head.getDetAngle().xyAngle());
            y = y() + b * Math.sin(head.getDetAngle().xyAngle());
            z = z() + b * Math.sin(head.getDetAngle().xzAngle());
            triangulations.add(new ThreeDPoint(x, y, z));
        }

        head.centroid = centerOf(triangulations); // write estimated location back to ObjDetection in this frame
    }

    public ThreeDPoint centerOf(ArrayList<ThreeDPoint> points) {
        double xavg = 0.0d;
        double yavg = 0.0d;
        double zavg = 0.0d;
        for (ThreeDPoint aPoint : points) {
            xavg += aPoint.x();
            yavg += aPoint.y();
            zavg += aPoint.z();
        }
        xavg /= points.size();
        yavg /= points.size();
        zavg /= points.size();
        return new ThreeDPoint(xavg, yavg, zavg);
    }

}
