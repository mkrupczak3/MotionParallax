package MotionParallax;

public class ThreeDAngle {
    // NED "North, East, Down" reference frame for x, y, z
    protected Double xyAngle; // bearing in the xy plane (radians). Starts from 0 = North and incresases clockwise eastward
    protected Double xzAngle; // bearing in the xz plane (radians). Starts from 0 = Forward and increases clockwise downward

    public ThreeDAngle(double xyAngle, double xzAngle) {
        if (xzAngle <= Math.PI / 2.0d || xyAngle >= 3.0d * Math.PI / 2.0d) { // xyAngle points towards target
            this.xyAngle = Utils.normalized(xyAngle);
            this.xzAngle = Utils.normalized(xzAngle);
        } else { // if xyAngle doesn't point towards target, reverse its direction and take the suplement of xzAngle
            this.xyAngle = Utils.normalized(xyAngle + Math.PI);
            this.xzAngle = Utils.normalized(Math.PI - xzAngle);
        }

    }

    public ThreeDAngle sumWith(ThreeDAngle other) {
        double xyAngle = Utils.normalized(this.xyAngle() + other.xyAngle());
        double xzAngle = Utils.normalized(this.xzAngle() + other.xzAngle());
        return new ThreeDAngle(xyAngle, xzAngle);
    }

    public double xyAngle() {
        return (double) xyAngle;
    }

    public double xzAngle() {
        return (double) xzAngle;
    }

    @Override
    public int hashCode() {
        return (xyAngle.hashCode() + xzAngle.hashCode());
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null) return false;
        if (this.getClass() != o.getClass()) return false;
        ThreeDAngle other = (ThreeDAngle) o;
        return (aboutEqual(this.xyAngle(), other.xyAngle()) && aboutEqual(this.xzAngle(), other.xzAngle()));
    }

    private boolean aboutEqual(double A, double B) {
        return (Math.abs(Double.compare(A, B)) < 0.0001d);
    }
}
