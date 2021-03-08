package org.chis.userclasses;

import java.awt.Color;
import java.util.ArrayList;

import org.chis.sim.*;
import org.chis.sim.Motor.MotorType;
import org.chis.sim.math.*;
import org.chis.sim.math.Vector2D.Type;
import org.chis.userclasses.SwerveController.Module;
import org.chis.userclasses.auto.ArcAction;
import org.chis.userclasses.auto.AutoSequence;
import org.chis.userclasses.auto.LineAction;;


public class UserCode{

    static ArrayList<Vector2D> trail = new ArrayList<Vector2D>();

    static GraphicDash robotLinVel = new GraphicDash("robotLinVel", 100, true);
    static GraphicDash robotAngVel = new GraphicDash("robotAngVel", 100, true);

    static final double offsetX = Constants.WHEEL_XDIST.getDouble();
    static final double offsetY = Constants.WHEEL_YDIST.getDouble();

    static Gyro gyro;
    static SwerveController swerve;

    static AutoSequence auto;

    public static void initialize(){ //this function is run once when the robot starts
        gyro = new Gyro();
        swerve = new SwerveController(
            gyro,
            new Module(1, 0, new Pose2D(+offsetX, +offsetY, Constants.WHEELANG0.getDouble())),
            new Module(3, 2, new Pose2D(-offsetX, +offsetY, Constants.WHEELANG1.getDouble())),
            new Module(5, 4, new Pose2D(-offsetX, -offsetY, Constants.WHEELANG2.getDouble())),
            new Module(7, 6, new Pose2D(+offsetX, -offsetY, Constants.WHEELANG3.getDouble()))
        );
        swerve.init(new Pose2D(Constants.INITX.getDouble(), Constants.INITY.getDouble(), Constants.INITANG.getDouble()));
        trail.clear();

        double speed = Constants.MAX_SPEED.getDouble();

        auto = new AutoSequence(
            swerve, 
            new LineAction(new Vector2D(-0.8, -0.3, Type.CARTESIAN), speed, 0.2),
            new ArcAction(new Vector2D(-0.8, -0.8, Type.CARTESIAN), speed, -1.8*Math.PI),
            new LineAction(new Vector2D(1.7, 0.3, Type.CARTESIAN), speed, 0.2),
            new ArcAction(new Vector2D(1.5, 0.8, Type.CARTESIAN), speed, 1.5*Math.PI),
            new LineAction(new Vector2D(2.5, -1.1, Type.CARTESIAN), speed, 0.2),
            new ArcAction(new Vector2D(3, -0.8, Type.CARTESIAN), speed, 1.15*Math.PI),
            new LineAction(new Vector2D(-3.5, 0, Type.CARTESIAN), speed, 0.2)
        );
    }

    public static void execute(){ //this function is run 50 times a second (every 0.02 second)

        //DRIVE CODE
        gyro.update();
        double heading = gyro.getContinuousAngle();
        Pose2D odoPose = swerve.odo.robotPose;

        Pose2D joystick = new Pose2D(Controls.rawX, -Controls.rawY, Controls.slider * -4);
        Pose2D targetRobotSpeeds = joystick.rotateVec(-heading).scalarMult(4);


        // swerve.nyoomToPoint(new Vector2D(0, 0, Type.CARTESIAN), 2);
        // swerve.nyoomAboutPoint(new Vector2D(-3.5, 1, Type.CARTESIAN), 2);
        // swerve.nyoom(targetRobotSpeeds);
        // swerve.nyoom(new Pose2D());

        auto.runSequence();


        // DEBUG
        trail.add(Main.robot.robotPos.getVector2D());
        GraphicSim.addDrawingGlobal(trail, Color.GREEN.darker());

        GraphicSim.updateOdometryDrawing(odoPose);

        robotLinVel.putNumber("tarX", swerve.targetSpeeds.x, Color.BLUE);
        robotLinVel.putNumber("tarY", swerve.targetSpeeds.y, Color.BLUE);
        robotAngVel.putNumber("tarAng", swerve.targetSpeeds.ang, Color.BLUE);

        robotLinVel.putNumber("curX", Main.robot.robotVel.rotate(-heading).x, Color.RED);
        robotLinVel.putNumber("curY", Main.robot.robotVel.rotate(-heading).y, Color.RED);
        robotAngVel.putNumber("curAng", Main.robot.robotVel.ang, Color.RED);

        Printouts.put("odoPose", odoPose);
        Printouts.put("auto", auto.action);
        Printouts.put("Elapsed Time", Main.elaspedTime);
    }

    public static double encoderToDist(double encoder){
        double driveGearRatio = 6.86;
        return encoder / MotorType.FALCON.TICKS_PER_REV / driveGearRatio * 2 * Math.PI * Constants.WHEEL_RADIUS.getDouble();
    }




}