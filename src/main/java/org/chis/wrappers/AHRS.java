package org.chis.wrappers;

import org.chis.sim.Main;
import org.chis.wrappers.SPI.Port;

public class AHRS {

    private double offset = 0;

    public AHRS(Port port){
        
    }

    public double getYaw(){
        return Main.robot.robotPos.ang - offset;
    }

    public double getRate(){
        return Main.robot.robotVel.ang;
    }

    public void reset(){
        offset = Main.robot.robotPos.ang;
    }
}
