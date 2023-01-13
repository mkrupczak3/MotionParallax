package MotionParallax;

class Utils {
    Utils() {
        super();
    }

    public static double normalize(double inAngle) {
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
        return(normalize(inAngle + Math.PI));
    }
}
