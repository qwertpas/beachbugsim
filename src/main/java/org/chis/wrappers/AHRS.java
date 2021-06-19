package org.chis.wrappers;

import org.chis.Main;
import org.chis.wrappers.SPI.Port;

public class AHRS {

    private double offset = 0;

    public AHRS(Port port){
        
    }

    /** @return angle of gyrocope in radians */
    public double getYaw(){
        return Main.robot.robotPos.ang - offset;
    }

    /** @return angle of gyrocope in radians per second */
    public double getRate(){
        return Main.robot.robotVel.ang;
    }

    /** zero the gyroscope */
    public void reset(){
        offset = Main.robot.robotPos.ang;
    }
}
