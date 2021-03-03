package org.chis.userclasses;

public class WheelData{
    double angle;
    double dist;
    public WheelData(double angle, double dist){
        this.angle = angle;
        this.dist = dist;
    }
    public String toString(){
        return "ang: " + angle + ", dist: " + dist;
    }
}