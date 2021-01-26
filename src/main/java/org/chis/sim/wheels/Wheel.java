package org.chis.sim.wheels;

import org.chis.sim.math.*;

public abstract class Wheel {

    public enum WheelType{
        CoaxSwerveModule, DiffSwerveModule, FixedWheel, OmniWheel;
    }
    public WheelType wheelType;
    
    public Pose2D placement; //pose relative to robot center
    public double wheelRadius;

    public Vector2D moduleTranslation;

    public double wheelRollVelo;
    public Vector2D force = new Vector2D();

    public abstract void update(Pose2D robotMovement, double dt);

    public void updateModuleTranslation(Pose2D robotMovement){
        Vector2D spinComponent = placement.getVector2D().rotate(Math.PI/2).scalarMult(robotMovement.ang); //part of module translation that comes from robot spinning
        Vector2D moduleTranslation_robot = spinComponent.add(robotMovement.getVector2D()); //entire module translation relative to robot reference frame
        moduleTranslation = moduleTranslation_robot.rotate(placement.ang); //get module translation relative to wheel reference frame

        // System.out.println("moduleRobot: " + moduleTranslation_robot);
        // System.out.println("spin: " + spinComponent);
    }

    

}
