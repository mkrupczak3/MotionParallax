import MotionParallax.HungarianAlgorithm;
import MotionParallax.ThreeDPoint;
import MotionParallax.ThreeDAngle;
import MotionParallax.ThreeDObjDetection;
import MotionParallax.ThreeDFrame;

public class FrameTest {

    public static void main(String[] args) {
        // constructor:
        // camera_bearing, x, y, double[] relative_bearings, Frame prev
        ThreeDFrame f0 = new ThreeDFrame(new ThreeDAngle(1.0d * Math.PI / 2.0d, 7.0d * Math.PI / 4.0d), new ThreeDPoint(100.0d, -100.0d, 100.0d), new ThreeDAngle [] {new ThreeDAngle(0.0d,0.0d)}, null);

        ThreeDFrame f1 = new ThreeDFrame(new ThreeDAngle(0.0d, 0.0d), new ThreeDPoint(0.0d, 0.0d, 0.0d), new ThreeDAngle [] {new ThreeDAngle(0.0d, 0.0d)}, f0);
        // ThreeDFrame f1 = new ThreeDFrame(new ThreeDAngle(0.0d, 0.0d), new ThreeDPoint(100.0d, 0.0d, 0.0d), new ThreeDAngle [] {new ThreeDAngle(7.0d * Math.PI / 4.0d, 0.0d)}, f0);

        f1.triangulate_all_objs();

        ThreeDPoint p = f1.get_all_objs().get(0).centroid;
        if (p == null) {
            System.out.println("got null");
        } else {
            System.out.printf("x: %f y: %f z: %f\n", p.x(), p.y(), p.z());
        }
    }
}
