package MotionParallax;

public class ThreeDPoint {
    protected double x;
    protected double y;
    protected double z;

    public ThreeDPoint(double x, double y, double z) {
        this.x = x;
        this.y = y;
        this.z = z;
    }

    public double x() {
        return x;
    }

    public double y() {
        return y;
    }

    public double z() {
        return z;
    }
}
