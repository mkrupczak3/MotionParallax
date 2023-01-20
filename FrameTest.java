import MotionParallax.HungarianAlgorithm;
import MotionParallax.Point;
import MotionParallax.ObjDetection;
import MotionParallax.Frame;

public class FrameTest {

    public static void main(String[] args) {
        Frame f0 = new Frame(0.0d, 0.0d, 0.0d, new double[] {(Math.PI / 4.0d)}, null);
        Frame f1 = new Frame(0.0d, 0.0d, 100.0d, new double[] {0.0d}, f0);
        System.out.println("got here");

        f1.correlateToPrev();
        System.out.println("correlated");
        f1.triangulate_all_objs();
        Point p = f1.get_all_objs().get(0).centroid;
        System.out.printf("x: %f z: %f\n", p.x(), p.z());
    }
}
