package org.chis.sim.wheels;

import org.chis.sim.Motor;
import org.chis.sim.Util;
import org.chis.sim.Motor.MotorType;
import org.chis.sim.math.*;
import org.chis.sim.math.Vector2D.Type;

public class CoaxSwerveModule extends Wheel {

    public Motor driveMotor, turnMotor;

    final double TURN_MOI;
    final double TURN_GR;
    final double ROLL_GR;

    public VerletIntegrator wheelTurnIntegrator = new VerletIntegrator(0, 0, 0, 0.1);

    public CoaxSwerveModule(
        Pose2D placement, 
        double wheelRadius, 
        Motor driveMotor, 
        Motor turnMotor,
        double TURN_MOI,
        double TURN_GR,
        double ROLL_GR
    ){
        this.placement = placement;
        this.wheelRadius = wheelRadius;
        this.driveMotor = driveMotor;
        this.turnMotor = turnMotor;
        this.TURN_MOI = TURN_MOI;
        this.TURN_GR = TURN_GR;
        this.ROLL_GR = ROLL_GR;
    }

    @Override
    public void update(Pose2D robotMovement, double dt){
    
        super.updateModuleTranslation(robotMovement);

        // update module angle
        double turnTorque = turnMotor.torque * TURN_GR;
        turnTorque = Util.applyFrictions(
            turnTorque, 
            wheelTurnIntegrator.vel, 
            60, 
            50, 
            0.1, 
            0.01
        );

        wheelTurnIntegrator.update(turnTorque / TURN_MOI, dt);
        turnMotor.update(wheelTurnIntegrator.vel / TURN_GR, dt);

        // System.out.println("vel: " + wheelTurnIntegrator.vel);




        // get angular velocity of wheel rolling and feed it to motor
        Vector2D wheelTranslation = moduleTranslation.rotate(-wheelTurnIntegrator.pos);
        wheelRollVelo = wheelTranslation.x / wheelRadius;
 
        driveMotor.update(wheelRollVelo * ROLL_GR, dt);


        
        // get motor and friction forces
        double driveTorque = Util.applyFrictions(
            driveMotor.torque * ROLL_GR,
            wheelRollVelo, 
            2,
            1.8,
            0,
            0.001
        );
        double driveForce = driveTorque / wheelRadius;

        double scrubForce = Util.applyFrictions(
            0,
            wheelTranslation.y,
            200,
            190,
            0,
            0.01
        );

        // add up motor and friction forces, then rotate by wheel angle to get force of module on robot
        force = new Vector2D(driveForce, scrubForce, Type.CARTESIAN)
            .rotate(wheelTurnIntegrator.pos) //make relative to module
            .rotate(placement.ang) //make relative to robot
        ;
    }

}
