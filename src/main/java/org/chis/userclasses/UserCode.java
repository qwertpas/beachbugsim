package org.chis.userclasses;

import java.awt.Color;
import java.util.ArrayList;

import org.chis.sim.*;
import org.chis.sim.Motor.MotorType;
import org.chis.sim.math.*;

import org.chis.userclasses.SwerveController.Module;;


public class UserCode{

    static ArrayList<Vector2D> trail = new ArrayList<Vector2D>();

    static GraphicDash moduleSpeeds = new GraphicDash("moduleSpeeds", 100, true);
    static GraphicDash moduleAngles = new GraphicDash("moduleAngles", 100, true);
    static GraphicDash robotSpeeds = new GraphicDash("robotspeeds", 100, true);
    

    static final double offsetX = Constants.WHEEL_XDIST.getDouble();
    static final double offsetY = Constants.WHEEL_YDIST.getDouble();

    static SwerveController swerve = new SwerveController(
        new Module(1, 0, new Pose2D(+offsetX, +offsetY, Constants.WHEELANG0.getDouble())),
        new Module(3, 2, new Pose2D(-offsetX, +offsetY, Constants.WHEELANG1.getDouble())),
        new Module(5, 4, new Pose2D(-offsetX, -offsetY, Constants.WHEELANG2.getDouble())),
        new Module(7, 6, new Pose2D(+offsetX, -offsetY, Constants.WHEELANG3.getDouble()))
    );

    public static void initialize(){ //this function is run once when the robot starts
        trail.clear();
    }

    public static void execute(){ //this function is run 50 times a second (every 0.02 second)


        //DRIVE CODE
        double heading = Main.robot.robotPos.ang;

        Pose2D joystick = new Pose2D(Controls.rawX, -Controls.rawY, Controls.slider * -0);

        Pose2D targetRobotSpeeds = joystick.rotateVec(-heading).scalarMult(4);

        swerve.move(targetRobotSpeeds);




        // DEBUG
        trail.add(Main.robot.robotPos.getVector2D());
        GraphicSim.addDrawingGlobal(trail, Color.GREEN.darker());

        for(int i = 0; i < swerve.modules.length; i++){
            moduleAngles.putNumber(i + "targetAngle", swerve.modules[i].targetAngle, Color.BLUE);
            moduleAngles.putNumber(i + "currentAngle", swerve.modules[i].currentAngle, Color.RED);

            moduleSpeeds.putNumber(i + "targetSpeed", swerve.modules[i].targetDriveSpeed, Color.BLUE);
            moduleSpeeds.putNumber(i + "currentSpeed", swerve.modules[i].currentDriveSpeed, Color.RED);
        }

        Vector2D robotCentricVel = Main.robot.robotVel.rotate(-heading);

        robotSpeeds.putNumber("vx target", targetRobotSpeeds.x, Color.BLUE);  
        robotSpeeds.putNumber("vx curr", robotCentricVel.x, Color.RED);  

        robotSpeeds.putNumber("vy target", targetRobotSpeeds.y, Color.BLUE);  
        robotSpeeds.putNumber("vy curr", robotCentricVel.y, Color.RED);  

        robotSpeeds.putNumber("ang target", targetRobotSpeeds.ang, Color.BLUE);  
        robotSpeeds.putNumber("ang curr", Main.robot.robotVel.ang, Color.RED);  
    }

    public static double encoderToDist(double encoder){
        double driveGearRatio = 6.86;
        return encoder / MotorType.FALCON.TICKS_PER_REV / driveGearRatio * 2 * Math.PI * Constants.WHEEL_RADIUS.getDouble();
    }




}