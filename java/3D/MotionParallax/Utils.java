package MotionParallax;

public class Utils {
    Utils() {
        super();
    }

    public static double normalized(double inAngle) {
        double outAngle = inAngle;
        while (outAngle >= 2.0d * Math.PI) {
            outAngle -= 2.0d * Math.PI;
        }
        while (outAngle < 0.0d) {
            outAngle += 2.0d * Math.PI;
        }
        return outAngle;
    }

    public static double getReverseAngle(double inAngle) {
        return normalized(inAngle + Math.PI);
    }

    public static ThreeDAngle normalized(ThreeDAngle inAngle) {
        return new ThreeDAngle(normalized(inAngle.xyAngle()), normalized(inAngle.xzAngle()));
    }

    public static ThreeDAngle getReverseAngle(ThreeDAngle inAngle) {
        return(new ThreeDAngle(getReverseAngle(inAngle.xyAngle()), getReverseAngle(inAngle.xzAngle())));
    }

    public static double getAngleDiff(ThreeDAngle A, ThreeDAngle B) {
        double phi = A.xyAngle();
        double theta = A.xzAngle();

        double phi_prime = B.xyAngle();
        double theta_prime = B.xzAngle();

        // find the angle of the shortest direct arc
        // https://math.stackexchange.com/a/2940458
        return normalized(Math.acos(cos(theta)*cos(theta_prime)*cos(phi-phi_prime) + sin(theta)*sin(theta_prime)));
    }

    public static double sin(double radAngle) {
        return Math.sin(radAngle);
    }

    public static double cos(double radAngle) {
        return Math.cos(radAngle);
    }
}
