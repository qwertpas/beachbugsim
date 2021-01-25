package org.chis.sim;

import org.chis.sim.Motor.MotorType;
import org.chis.sim.math.*;
import org.chis.sim.wheels.CoaxSwerveModule;
import org.chis.sim.wheels.Wheel;

//overarching physics simulation
public class Robot{

    public Wheel[] wheels = {
        new CoaxSwerveModule(
            new Pose2D(1, 1, 0), 
            Constants.WHEEL_RADIUS.getDouble(), 
            new Motor(MotorType.FALCON, 1), 
            new Motor(MotorType.FALCON, 1), 
            Constants.SWERVE_MOI.getDouble(), 
            12.8, 
            6.86
        ),
        new CoaxSwerveModule(
            new Pose2D(-1, 1, 0), 
            Constants.WHEEL_RADIUS.getDouble(), 
            new Motor(MotorType.FALCON, 1), 
            new Motor(MotorType.FALCON, 1), 
            Constants.SWERVE_MOI.getDouble(), 
            12.8, 
            6.86
        ),
        new CoaxSwerveModule(
            new Pose2D(-1, -1, 0), 
            Constants.WHEEL_RADIUS.getDouble(), 
            new Motor(MotorType.FALCON, 1), 
            new Motor(MotorType.FALCON, 1), 
            Constants.SWERVE_MOI.getDouble(), 
            12.8, 
            6.86
        ),
        new CoaxSwerveModule(
            new Pose2D(1, -1, 0), 
            Constants.WHEEL_RADIUS.getDouble(), 
            new Motor(MotorType.FALCON, 1), 
            new Motor(MotorType.FALCON, 1), 
            Constants.SWERVE_MOI.getDouble(), 
            12.8, 
            6.86
        ),
    };

    //dynamics on the whole robot
    public Vector2D netForce = new Vector2D();
    public double netTorque = 0;

    //robot state in meters, radians, and seconds
    public Pose2D robotPos = new Pose2D();
    public Pose2D robotVel = new Pose2D();
    public Pose2D robotAcc = new Pose2D();

    //integrators
    VerletIntegrator xIntegrator = new VerletIntegrator(0, 0, 0, Constants.DT.getDouble());
    VerletIntegrator yIntegrator = new VerletIntegrator(0, 0, 0, Constants.DT.getDouble());
    VerletIntegrator angIntegrator = new VerletIntegrator(0, 0, 0, Constants.DT.getDouble());

    public void init(){

        
        // leftGearbox.setPower(0);
        // rightGearbox.setPower(0);

        // for(Motor motor : leftGearbox.motors){
        //     motor.resetEncoder();
        // }
        // for(Motor motor : rightGearbox.motors){
        //     motor.resetEncoder();
        // }
    }

    public void update(double dt){

        netForce = new Vector2D();
        netTorque = 0;
        for(Wheel wheel : wheels){
            wheel.update(robotVel, dt);
            netForce = netForce.add(wheel.force);
            netTorque = netTorque + wheel.force.pcross(wheel.placement.getVector2D());
        }

        //wheels scrub, applying a frictional torque that slows turning
        netTorque = Util.applyFrictions(
            netTorque, 
            robotVel.ang, 
            Constants.SCRUB_STATIC, 
            Constants.SCRUB_KINE, 
            0, 
            Constants.ANGVELO_THRESHOLD.getDouble())
        ;

        //Newton's 2nd Law to find accelerations given forces and torques
        robotAcc.setVector2D(netForce.scalarDiv(Constants.ROBOT_MASS.getDouble()));
        robotAcc.ang = netTorque / Constants.ROBOT_ROT_INERTIA;

        xIntegrator.update(robotAcc.x, dt);
        yIntegrator.update(robotAcc.y, dt);
        angIntegrator.update(robotAcc.ang, dt);

        robotVel = new Pose2D(xIntegrator.vel, yIntegrator.vel, angIntegrator.vel);
        robotPos = new Pose2D(xIntegrator.pos, yIntegrator.pos, angIntegrator.pos);
    }


    void printDebug(){
        System.out.println("Pos: " + robotPos);
        System.out.println("Vel: " + robotVel);
        System.out.println("Acc: " + robotAcc);
        System.out.println("netForce: " + netForce);
        System.out.println("netTorque: " + netTorque);
        System.out.println("——————————————————");
    }
    


}