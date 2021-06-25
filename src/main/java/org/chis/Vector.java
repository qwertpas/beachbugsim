package org.chis;

public class Vector {
    double x;
    double y;

    public Vector(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public Vector setX(double x) {
        this.x = x;
        return this;
    }

    public Vector setY(double y) {
        this.y = y;
        return this;
    }

    public double magnitude() {
        return Math.sqrt(x*x + y*y);
    }

    public Vector add(Vector a) {
        return new Vector(x+a.x, y+a.y);
    }

    public double getAngleDeg() {
        return Math.toDegrees(Math.atan2(y, x));
        // tan theta = y/x
        // theta = atan2(y/x)
    }

    public static Vector angleMagTranslation(double theta, double magnitude) {
        Vector n =  new Vector(0,0);

        double angle = Math.toRadians(theta);
        double x = magnitude * Math.sin(angle);
        double y = magnitude * Math.cos(angle);

        return n.setX(x).setY(y);

        // x^2 + b^2 = magnitude^2
        // sin(theta) = a/magnitude
        // a = magnitude*sin(theta)
        // cos(theta) = b/magnitude
        // b = magnitude*cos(theta)
    }
}
