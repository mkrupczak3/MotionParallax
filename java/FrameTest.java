import MotionParallax.HungarianAlgorithm;
import MotionParallax.Point;
import MotionParallax.ObjDetection;
import MotionParallax.Frame;

public class FrameTest {

    public static void main(String[] args) {
        // constructor:
        // camera_bearing, x, y, double[] relative_bearings, Frame prev
        Frame f0 = new Frame(0.0d, 0.0d, 0.0d, new double[] {(Math.PI / 4.0d)}, null);
        Frame f1 = new Frame(0.0d, 0.0d, 100.0d, new double[] {0.0d}, f0);

        f1.correlateToPrev();
        System.out.println("correlated");
        f1.triangulate_all_objs();
        System.out.println("triangulated");

        Point p = f1.get_all_objs().get(0).centroid;
        System.out.printf("x: %f y: %f\n", p.x(), p.y());
    }
}
