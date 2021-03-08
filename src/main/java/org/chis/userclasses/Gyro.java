package org.chis.userclasses;

import org.chis.sim.Main;

public class Gyro {

    double offset = 0;

    double getAngle(){
        return Main.robot.robotPos.ang - offset;
    }

    void zero(){
        offset = Main.robot.robotPos.ang;
    }
}
