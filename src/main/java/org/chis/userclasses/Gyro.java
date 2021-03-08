package org.chis.userclasses;

import org.chis.sim.Main;
import org.chis.sim.Util;

public class Gyro {

    double offset = 0;
    double lastRawAngle = 0;
    double rotations = 0;

    public void update(){
        double currRawAngle = getRawAngle();

        if(Util.between(lastRawAngle, 1.5*Math.PI, 2*Math.PI) && Util.between(currRawAngle, 0, 0.5*Math.PI)){
            rotations++;
            System.out.println("aaa");
        }else if(Util.between(lastRawAngle, 0, 0.5*Math.PI) && Util.between(currRawAngle, 1.5*Math.PI, 2*Math.PI)){
            rotations--;
        }
        lastRawAngle = currRawAngle;
    }

    //gyro usually returns angle between 0 and 2PI
    public double getRawAngle(){
        return Util.posModulo(Main.robot.robotPos.ang - offset, 2*Math.PI);
    }

    //making angle continuous by allowing it to go below 0 or above 2PI
    public double getContinuousAngle(){
        return rotations * 2*Math.PI + getRawAngle();
    }

    public void zero(){
        offset = Main.robot.robotPos.ang;
    }
}
