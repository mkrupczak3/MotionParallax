package MotionParallax;

class Utils {
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

    public static double getReverseBearing(double inAngle) {
        return(normalized(inAngle + Math.PI));
    }

    public static double getAngleDiff(double A, double B) {
        double diff = Math.abs(normalized(A - B));
        if (diff > Math.PI) { // if angle is a reflex angle, get its smaller equivalent
            diff = 2.0d * Math.PI - diff;
        }
        return diff;
    }
}
