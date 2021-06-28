package org.chis;

public class Vector {
    double x;
    double y;
    double magnitude;


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

    public Vector setMagnitude(double m) {
        this.magnitude = m;
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
    }

    public static Vector angleMagTranslation(double theta_deg, double magnitude) {
        Vector n =  new Vector(0,0);

        // if (magnitude < 0) {
        //     theta_deg += 180;
        // }

        double theta_rad = Math.toRadians(theta_deg);
        double x = magnitude * Math.cos(theta_rad);
        double y = magnitude * Math.sin(theta_rad);

        return n.setX(x).setY(y);

        // x^2 + b^2 = magnitude^2
        // sin(theta) = a/magnitude
        // a = magnitude*sin(theta)
        // cos(theta) = b/magnitude
        // b = magnitude*cos(theta)
    }
}
