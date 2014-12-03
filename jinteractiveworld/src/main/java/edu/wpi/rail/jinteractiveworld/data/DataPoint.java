package edu.wpi.rail.jinteractiveworld.data;


/**
 * A DataPoint contains a position and rotation.
 *
 * @author Russell Toris -- rctoris@wpi.edu
 * @version December 3, 2014
 */
public class DataPoint {

    private double x, y, z, theta;

    public DataPoint() {
        this(0, 0, 0, 0);
    }

    public DataPoint(double x, double y, double z, double theta) {
        this.x = x;
        this.y = y;
        this.z = z;
        this.theta = theta;
    }

    public double getX() {
        return x;
    }

    public void setX(double x) {
        this.x = x;
    }

    public double getY() {
        return y;
    }

    public void setY(double y) {
        this.y = y;
    }

    public double getZ() {
        return z;
    }

    public void setZ(double z) {
        this.z = z;
    }

    public double getTheta() {
        return theta;
    }

    public void setTheta(double theta) {
        this.theta = theta;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;

        DataPoint dataPoint = (DataPoint) o;

        if (Double.compare(dataPoint.theta, theta) != 0) return false;
        if (Double.compare(dataPoint.x, x) != 0) return false;
        if (Double.compare(dataPoint.y, y) != 0) return false;
        if (Double.compare(dataPoint.z, z) != 0) return false;

        return true;
    }

    @Override
    public int hashCode() {
        int result;
        long temp;
        temp = Double.doubleToLongBits(x);
        result = (int) (temp ^ (temp >>> 32));
        temp = Double.doubleToLongBits(y);
        result = 31 * result + (int) (temp ^ (temp >>> 32));
        temp = Double.doubleToLongBits(z);
        result = 31 * result + (int) (temp ^ (temp >>> 32));
        temp = Double.doubleToLongBits(theta);
        result = 31 * result + (int) (temp ^ (temp >>> 32));
        return result;
    }
}
