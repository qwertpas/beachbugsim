package org.chis.sim;

import java.util.ArrayList;

import org.chis.sim.Motor.MotorType;
import org.chis.sim.math.*;
import org.chis.sim.wheels.CoaxSwerveModule;
import org.chis.sim.wheels.Wheel;

//overarching physics simulation
public class Robot{

    public ArrayList<Motor> motors;

    public Wheel[] wheels;

    //dynamics on the whole robot
    public Vector2D netForce;
    public double netTorque;

    //robot state in meters, radians, and seconds
    public Pose2D robotPos, robotVel, robotAcc;

    VerletIntegrator xIntegrator, yIntegrator, angIntegrator;

    public void init(){

        motors = new ArrayList<Motor>();
        for(int i = 0; i <= 16; i++){
            motors.add(new Motor(MotorType.FALCON, 1));
        }

        double wheelX = Constants.WHEEL_XDIST.getDouble();
        double wheelY = Constants.WHEEL_YDIST.getDouble();

        double turnRatio = 12.8;
        double driveRatio = 6.86;

        wheels = new Wheel[] {
            new CoaxSwerveModule(
                new Pose2D(wheelX, wheelY, 0), 
                Constants.WHEEL_RADIUS.getDouble(), 
                motors.get(0), 
                motors.get(1), 
                Constants.SWERVE_MOI.getDouble(), 
                turnRatio, 
                driveRatio
            ),
            new CoaxSwerveModule(
                new Pose2D(-wheelX, wheelY, 0), 
                Constants.WHEEL_RADIUS.getDouble(), 
                motors.get(2), 
                motors.get(3), 
                Constants.SWERVE_MOI.getDouble(), 
                turnRatio, 
                driveRatio
            ),
            new CoaxSwerveModule(
                new Pose2D(-wheelX, -wheelY, 0), 
                Constants.WHEEL_RADIUS.getDouble(), 
                motors.get(4), 
                motors.get(5), 
                Constants.SWERVE_MOI.getDouble(), 
                turnRatio, 
                driveRatio
            ),
            new CoaxSwerveModule(
                new Pose2D(wheelX, -wheelY, 0), 
                Constants.WHEEL_RADIUS.getDouble(), 
                motors.get(6), 
                motors.get(7), 
                Constants.SWERVE_MOI.getDouble(), 
                turnRatio, 
                driveRatio
            ),
        };

        netForce = new Vector2D();
        netTorque = 0;

        xIntegrator = new VerletIntegrator(0, 0, 0, Constants.PHYSICS_DT.getDouble());
        yIntegrator = new VerletIntegrator(0, 0, 0, Constants.PHYSICS_DT.getDouble());
        angIntegrator = new VerletIntegrator(0, 0, 0, Constants.PHYSICS_DT.getDouble());

        robotPos = new Pose2D();
        robotVel = new Pose2D();
        robotAcc = new Pose2D();
    }

    public void update(double dt){

        netForce = new Vector2D();
        netTorque = 0;
        for(Wheel wheel : wheels){
            Pose2D robotVel_robot = new Pose2D(robotVel.getVector2D().rotate(-robotPos.ang), robotVel.ang);
            wheel.update(robotVel_robot, dt);
            netForce = netForce.add(wheel.force);
            netTorque = netTorque + wheel.force.pcross(wheel.placement.getVector2D());
        }
        netForce = netForce.rotate(robotPos.ang);

        //Newton's 2nd Law to find accelerations given forces and torques
        robotAcc.setVector2D(netForce.scalarDiv(Constants.ROBOT_MASS.getDouble()));

        double ROBOT_ROT_INERTIA = Constants.ROBOT_MASS.getDouble() * Constants.WHEEL_XDIST.getDouble() * Constants.WHEEL_YDIST.getDouble() / 6.0;
        robotAcc.ang = netTorque / ROBOT_ROT_INERTIA;

        xIntegrator.update(robotAcc.x, dt);
        yIntegrator.update(robotAcc.y, dt);
        angIntegrator.update(robotAcc.ang, dt);

        robotVel = new Pose2D(xIntegrator.vel, yIntegrator.vel, angIntegrator.vel);
        robotPos = new Pose2D(xIntegrator.pos, yIntegrator.pos, angIntegrator.pos);
    }



}