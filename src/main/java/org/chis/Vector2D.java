package org.chis;

public class Vector2D{
    public double x;
    public double y;

    public Vector2D(double x, double y){
        this.x = x;
        this.y = y;
    }

    public double getMagnitude(){
        return Math.hypot(x, y);
    }

    public double getAngle(){
        return Math.atan2(y, x);
    }

    public Vector2D addVector(Vector2D addend){
        return new Vector2D(x + addend.x, y + addend.y);
    }

    public Vector2D multScalar(double scalar){
        return new Vector2D(x * scalar, y * scalar);
    }

    public Vector2D rotate90(){
        return new Vector2D(-y, x);
    }

    public Vector2D rotateGF(double radiansToRotate) {
        double sin = Math.sin(radiansToRotate);
        double cos = Math.cos(radiansToRotate);
        return new Vector2D(this.x * cos - this.y * sin, this.x * sin + this.y * cos);
    }
}