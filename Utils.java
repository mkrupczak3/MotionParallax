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
        return Math.abs(normalized(A - B));
    }
}
