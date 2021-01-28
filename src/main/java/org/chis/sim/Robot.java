package org.chis.sim;

import org.chis.sim.Motor.MotorType;
import org.chis.sim.math.*;
import org.chis.sim.wheels.CoaxSwerveModule;
import org.chis.sim.wheels.Wheel;

//overarching physics simulation
public class Robot{

    public Wheel[] wheels;

    //dynamics on the whole robot
    public Vector2D netForce = new Vector2D();
    public double netTorque = 0;

    //robot state in meters, radians, and seconds
    public Pose2D robotPos = new Pose2D();
    public Pose2D robotVel = new Pose2D();
    public Pose2D robotAcc = new Pose2D();

    //integrators
    VerletIntegrator xIntegrator = new VerletIntegrator(0, 0, 0, Constants.PHYSICS_DT.getDouble());
    VerletIntegrator yIntegrator = new VerletIntegrator(0, 0, 0, Constants.PHYSICS_DT.getDouble());
    VerletIntegrator angIntegrator = new VerletIntegrator(0, 0, 0, Constants.PHYSICS_DT.getDouble());

    public void init(){
        wheels = new Wheel[] {
            new CoaxSwerveModule(
                new Pose2D(0.3302, 0.3302, 0), 
                Constants.WHEEL_RADIUS.getDouble(), 
                new Motor(MotorType.FALCON, 1), 
                new Motor(MotorType.FALCON, 1), 
                Constants.SWERVE_MOI.getDouble(), 
                12.8, 
                6.86
            ),
            new CoaxSwerveModule(
                new Pose2D(-0.3302, 0.3302, 0), 
                Constants.WHEEL_RADIUS.getDouble(), 
                new Motor(MotorType.FALCON, 1), 
                new Motor(MotorType.FALCON, 1), 
                Constants.SWERVE_MOI.getDouble(), 
                12.8, 
                6.86
            ),
            new CoaxSwerveModule(
                new Pose2D(-0.3302, -0.3302, 0), 
                Constants.WHEEL_RADIUS.getDouble(), 
                new Motor(MotorType.FALCON, 1), 
                new Motor(MotorType.FALCON, 1), 
                Constants.SWERVE_MOI.getDouble(), 
                12.8, 
                6.86
            ),
            new CoaxSwerveModule(
                new Pose2D(0.3302, -0.3302, 0), 
                Constants.WHEEL_RADIUS.getDouble(), 
                new Motor(MotorType.FALCON, 1), 
                new Motor(MotorType.FALCON, 1), 
                Constants.SWERVE_MOI.getDouble(), 
                12.8, 
                6.86
            ),
        };
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

        double ROBOT_ROT_INERTIA = Constants.ROBOT_MASS.getDouble() * Constants.ROBOT_WIDTH.getDouble() * Constants.ROBOT_WIDTH.getDouble() / 6.0;
        robotAcc.ang = netTorque / ROBOT_ROT_INERTIA;

        xIntegrator.update(robotAcc.x, dt);
        yIntegrator.update(robotAcc.y, dt);
        angIntegrator.update(robotAcc.ang, dt);

        robotVel = new Pose2D(xIntegrator.vel, yIntegrator.vel, angIntegrator.vel);
        robotPos = new Pose2D(xIntegrator.pos, yIntegrator.pos, angIntegrator.pos);
    }



}