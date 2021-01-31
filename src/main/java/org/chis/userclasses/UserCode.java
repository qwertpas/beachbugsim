package org.chis.userclasses;

import java.awt.Color;
import java.util.ArrayList;

import org.chis.sim.*;
import org.chis.sim.Motor.MotorType;
import org.chis.sim.math.*;

import org.chis.userclasses.SwerveController.Module;;


public class UserCode{

    static ArrayList<Vector2D> trail = new ArrayList<Vector2D>();

    // static GraphicDash dynamics = new GraphicDash("dynamics", 100, true);
    // static GraphicDash state = new GraphicDash("state", 100, true);

    static GraphicDash module0angle = new GraphicDash("module0angle", 100, true);
    static GraphicDash module0drive = new GraphicDash("module0drive", 100, true);
    static GraphicDash speeds = new GraphicDash("speeds", 100, true);
    static GraphicDash robotSpeeds = new GraphicDash("robotSpeeds", 100, true);
    

    static final double offsetX = Constants.WHEEL_XDIST.getDouble();
    static final double offsetY = Constants.WHEEL_YDIST.getDouble();

    static SwerveController swerve = new SwerveController(
        new Module(1, 0, new Pose2D(+offsetX, +offsetY, 0)),
        new Module(3, 2, new Pose2D(-offsetX, +offsetY, 0)),
        new Module(5, 4, new Pose2D(-offsetX, -offsetY, 0)),
        new Module(7, 6, new Pose2D(+offsetX, -offsetY, 0))
    );

    public static void initialize(){ //this function is run once when the robot starts
        
    }

    public static void execute(){ //this function is run 50 times a second (every 0.02 second)

        double heading = Main.robot.robotPos.ang;

        Pose2D joystick = new Pose2D(Controls.rawX, -Controls.rawY, Controls.rawZ);
        Pose2D targetRobotSpeeds = joystick.rotateVec(-heading).scalarMult(2);


        // Pose2D targetRobotSpeeds = new Pose2D(2, 2, -1);


        swerve.move(targetRobotSpeeds);

        // Main.robot.motors.get(1).setPower(1);
        // Main.robot.motors.get(3).setPower(1);
        // Main.robot.motors.get(5).setPower(1);
        // Main.robot.motors.get(7).setPower(1);

        // Main.robot.motors.get(1).setPower(0.4);


        trail.add(Main.robot.robotPos.getVector2D());
        GraphicSim.addDrawingGlobal(trail, Color.GREEN.darker());

        //printing values in separate window
        // dynamics.putNumber("netForceMag", Main.robot.netForce.getMagnitude(), Color.RED);
        // dynamics.putNumber("netTorque", Main.robot.netTorque, Color.GREEN.darker());

        module0angle.putNumber("targetAngle", swerve.modules[0].targetAngleOptimized, Color.BLUE);
        module0angle.putNumber("currentAngle", swerve.modules[0].currentAngle, Color.RED);
        module0angle.putNumber("turnPower", swerve.modules[0].turnPower, Color.GREEN);

        module0drive.putNumber("targetSpeed", swerve.modules[0].targetDriveSpeed, Color.BLUE);
        module0drive.putNumber("currentSpeed", swerve.modules[0].currentDriveSpeed, Color.RED);
        module0drive.putNumber("drivePower", swerve.modules[0].drivePower, Color.GREEN);

        speeds.putNumber("0", swerve.modules[0].currentDriveSpeed, Color.BLACK);
        speeds.putNumber("1", swerve.modules[1].currentDriveSpeed, Color.BLACK);
        speeds.putNumber("2", swerve.modules[2].currentDriveSpeed, Color.BLACK);
        speeds.putNumber("3", swerve.modules[3].currentDriveSpeed, Color.BLACK);

        robotSpeeds.putNumber("currentAngvelo", Main.robot.robotVel.ang, Color.RED);
        robotSpeeds.putNumber("targetDrive", swerve.modules[0].targetDriveSpeed, Color.magenta);
        robotSpeeds.putNumber("targetAngvelo", targetRobotSpeeds.ang, Color.BLUE);

        Printouts.put("voltage0", Main.robot.motors.get(0).voltage);
        Printouts.put("encoder1pos", Main.robot.motors.get(1).position);
        

        Printouts.put("x", Main.robot.robotPos.x);
        Printouts.put("y", Main.robot.robotPos.y);
        Printouts.put("Heading", Main.robot.robotPos.ang);
    

    }

    public static double encoderToDist(double encoder){
        double driveGearRatio = 6.86;
        return encoder / MotorType.FALCON.TICKS_PER_REV / driveGearRatio * 2 * Math.PI * Constants.WHEEL_RADIUS.getDouble();
    }




}