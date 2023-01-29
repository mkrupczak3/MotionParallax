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
        double xyDiff = normalized(B.xyAngle() - A.xyAngle());
        double xzDiff = normalized(B.xzAngle() - A.xzAngle());

        if (xyDiff > Math.PI) { // if angle is a reflex angle, get its smaller equivalent
            xyDiff = 2.0d * Math.PI - xyDiff;
        }

        if (xzDiff > Math.PI) { // if angle is a reflex angle, get its smaller equivalent
            xzDiff = 2.0d * Math.PI - xzDiff;
        }

        // pythagorean formula for sphere: cos(arclen a) = cos(arclen b) * cos(arclen c)
        // find the angle of the shortest direct arc
        return normalized(Math.acos(Math.cos(xyDiff) * Math.cos(xzDiff)));
    }
}
