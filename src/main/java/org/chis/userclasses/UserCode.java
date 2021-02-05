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

    // static GraphicDash module0angle = new GraphicDash("module0angle", 100, true);
    // static GraphicDash module0drive = new GraphicDash("module0drive", 100, true);
    static GraphicDash speeds = new GraphicDash("speeds", 100, true);
    static GraphicDash angles = new GraphicDash("angles", 100, true);

    static GraphicDash robotSpeeds = new GraphicDash("angles", 100, true);
    

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

        Pose2D joystick = new Pose2D(Controls.rawX, -Controls.rawY, 0);
        Pose2D targetRobotSpeeds = joystick.rotateVec(-heading).scalarMult(2);


        // Pose2D targetRobotSpeeds = new Pose2D(1, 0, 0);


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

        for(int i = 0; i < swerve.modules.length; i++){
            angles.putNumber("targetAngle", swerve.modules[i].targetAngleOptimized, Color.BLUE);
            angles.putNumber("currentAngle", swerve.modules[i].currentAngle, Color.RED);

            speeds.putNumber("targetSpeed", swerve.modules[i].targetDriveSpeed, Color.BLUE);
            speeds.putNumber("currentSpeed", swerve.modules[i].currentDriveSpeed, Color.RED);
        }

        robotSpeeds.putNumber("vx target", targetRobotSpeeds.x, Color.BLUE);  
        robotSpeeds.putNumber("vx curr", Main.robot.robotVel.x, Color.RED);  

        robotSpeeds.putNumber("vy target", targetRobotSpeeds.y, Color.BLUE);  
        robotSpeeds.putNumber("vy curr", Main.robot.robotVel.y, Color.RED);  

        robotSpeeds.putNumber("ang target", targetRobotSpeeds.ang, Color.BLUE);  
        robotSpeeds.putNumber("ang curr", Main.robot.robotVel.ang, Color.RED);  

        Printouts.put("x", Main.robot.robotPos.x);
        Printouts.put("y", Main.robot.robotPos.y);
        Printouts.put("Heading", Main.robot.robotPos.ang);
    

    }

    public static double encoderToDist(double encoder){
        double driveGearRatio = 6.86;
        return encoder / MotorType.FALCON.TICKS_PER_REV / driveGearRatio * 2 * Math.PI * Constants.WHEEL_RADIUS.getDouble();
    }




}