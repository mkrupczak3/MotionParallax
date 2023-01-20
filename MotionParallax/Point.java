package MotionParallax;

class Point {
    protected double x;
    protected double z;

    Point(double x, double z) {
        this.x = x;
        this.z = z;
    }

    public double x() {
        return x;
    }

    public double z() {
        return z;
    }
}
