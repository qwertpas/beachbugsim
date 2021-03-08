package org.chis.userclasses;

import java.awt.Color;
import java.util.ArrayList;

import org.chis.sim.*;
import org.chis.sim.Motor.MotorType;
import org.chis.sim.math.*;
import org.chis.sim.math.Vector2D.Type;
import org.chis.userclasses.SwerveController.Module;;


public class UserCode{

    static ArrayList<Vector2D> trail = new ArrayList<Vector2D>();

    static GraphicDash robotLinVel = new GraphicDash("robotLinVel", 100, true);
    static GraphicDash robotAngVel = new GraphicDash("robotAngVel", 100, true);

    static final double offsetX = Constants.WHEEL_XDIST.getDouble();
    static final double offsetY = Constants.WHEEL_YDIST.getDouble();

    static SwerveController swerve;

    public static void initialize(){ //this function is run once when the robot starts
        swerve = new SwerveController(
            new Gyro(),
            new Module(1, 0, new Pose2D(+offsetX, +offsetY, Constants.WHEELANG0.getDouble())),
            new Module(3, 2, new Pose2D(-offsetX, +offsetY, Constants.WHEELANG1.getDouble())),
            new Module(5, 4, new Pose2D(-offsetX, -offsetY, Constants.WHEELANG2.getDouble())),
            new Module(7, 6, new Pose2D(+offsetX, -offsetY, Constants.WHEELANG3.getDouble()))
        );
        swerve.init(new Pose2D(Constants.INITX.getDouble(), Constants.INITY.getDouble(), Constants.INITANG.getDouble()));
        trail.clear();
    }

    public static void execute(){ //this function is run 50 times a second (every 0.02 second)


        //DRIVE CODE
        double heading = swerve.gyro.getAngle();
        Pose2D odoPose = swerve.odo.robotPose;

        Pose2D joystick = new Pose2D(Controls.rawX, -Controls.rawY, Controls.slider * -4);
        Pose2D targetRobotSpeeds = joystick.rotateVec(-heading).scalarMult(4);


        swerve.nyoomToPoint(new Vector2D(0, 0, Type.CARTESIAN), 2);
        // swerve.nyoom(targetRobotSpeeds);




        // DEBUG
        trail.add(Main.robot.robotPos.getVector2D());
        GraphicSim.addDrawingGlobal(trail, Color.GREEN.darker());

        GraphicSim.updateOdometryDrawing(odoPose);

        robotLinVel.putNumber("tarX", swerve.targetSpeeds.x, Color.BLUE);
        robotLinVel.putNumber("tarY", swerve.targetSpeeds.y, Color.BLUE);
        robotAngVel.putNumber("tarAng", swerve.targetSpeeds.ang, Color.BLUE);

        robotLinVel.putNumber("curX", Main.robot.robotVel.rotate(Main.robot.robotVel.ang).x, Color.RED);
        robotLinVel.putNumber("curY", Main.robot.robotVel.rotate(Main.robot.robotVel.ang).y, Color.RED);
        robotAngVel.putNumber("curAng", Main.robot.robotVel.ang, Color.RED);

        Printouts.put("odopose", swerve.odo.robotPose);
        Printouts.put("encoder ang", swerve.modules[0].currentAngle - swerve.modules[0].lastAngle);
        Printouts.put("encoder dist", swerve.modules[0].currentDrivePos - swerve.modules[0].lastDrivePos);

    }

    public static double encoderToDist(double encoder){
        double driveGearRatio = 6.86;
        return encoder / MotorType.FALCON.TICKS_PER_REV / driveGearRatio * 2 * Math.PI * Constants.WHEEL_RADIUS.getDouble();
    }




}