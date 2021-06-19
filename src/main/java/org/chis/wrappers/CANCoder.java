package org.chis.wrappers;

import org.chis.Main;
import org.chis.sim.Motor;
import org.chis.sim.wheels.CoaxSwerveModule;

public class CANCoder {

    int moduleID;
    Motor motor;
    boolean absolute = false;

    public CANCoder(int deviceNumber){
        moduleID = deviceNumber - 9;
        motor = Main.robot.motors.get(moduleID * 2 + 1);
    }

    public void configSensorInitializationStrategy(SensorInitializationStrategy initializationStrategy){
        if(initializationStrategy == SensorInitializationStrategy.BootToAbsolutePosition){
            absolute = true;
        }
    }

    public double getPosition(){
        CoaxSwerveModule module = (CoaxSwerveModule) Main.robot.wheels[moduleID];

        if(absolute){
            return Math.toDegrees(module.wheelTurnIntegrator.pos + module.placement.ang);
        }else{
            return Math.toDegrees(module.wheelTurnIntegrator.pos);
        }
    }

    public double getVelocity(){
        CoaxSwerveModule module = (CoaxSwerveModule) Main.robot.wheels[moduleID];

        return Math.toDegrees(module.wheelTurnIntegrator.vel);
    }

}
