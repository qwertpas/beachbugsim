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

    static SwerveController swerve = new SwerveController(
        new Module(1, 0, new Pose2D(+offsetX, +offsetY, Constants.WHEELANG0.getDouble())),
        new Module(3, 2, new Pose2D(-offsetX, +offsetY, Constants.WHEELANG1.getDouble())),
        new Module(5, 4, new Pose2D(-offsetX, -offsetY, Constants.WHEELANG2.getDouble())),
        new Module(7, 6, new Pose2D(+offsetX, -offsetY, Constants.WHEELANG3.getDouble()))
    );

    static int step = 0;

    public static void initialize(){ //this function is run once when the robot starts
        swerve.odo.robotPose = new Pose2D();
        trail.clear();
    }

    public static void execute(){ //this function is run 50 times a second (every 0.02 second)


        //DRIVE CODE
        double heading = Main.robot.robotPos.ang;
        Pose2D robotPose = swerve.odo.robotPose;

        Pose2D joystick = new Pose2D(Controls.rawX, -Controls.rawY, Controls.slider * -4);
        Pose2D targetRobotSpeeds = joystick.rotateVec(-heading).scalarMult(4);

        // Pose2D targetRobotSpeeds = new Pose2D();
        // double v = 2;

        // if(step == 0){
        //     targetRobotSpeeds = new Pose2D(0.01, 0, 0); //point wheels
        //     if(Main.elaspedTime > 0.2) step++;
        // }
        // if(step == 1){
        //     targetRobotSpeeds = new Pose2D(v, 0, 0); //right
        //     if(robotPose.x > 3) step++;
        // }
        // if(step == 2){
        //     targetRobotSpeeds = new Pose2D(0, -v, 0); //down
        //     if(robotPose.y < -1) step++;
        // }
        // if(step == 3){
        //     targetRobotSpeeds = new Pose2D(-v, 0, 0); //left
        //     if(robotPose.x < 2) step=4;
        // }
        // if(step == 4){
        //     targetRobotSpeeds = new Pose2D(0, v, 0); //up
        //     Printouts.put("4", robotPose.y);
        //     // if(robotPose.y > 2) step++;
        // }
        // if(step == 5){
        //     targetRobotSpeeds = new Pose2D(v, 0, 0); //right
        //     if(robotPose.x > 6) step++;
        // }
        // if(step == 6){
        //     targetRobotSpeeds = new Pose2D(0, v, 0); //up
        //     if(robotPose.y > 2) step++;
        // }
        // if(step == 7){
        //     targetRobotSpeeds = new Pose2D(-v, 0, 0); //left
        //     if(robotPose.x < 5) step++;
        // }
        // if(step == 4){
        //     targetRobotSpeeds = new Pose2D(0, -v, 0); //down
        //     if(robotPose.y < 0) step++;
        // }
        // Printouts.put("step", step);




        swerve.nyoom(targetRobotSpeeds);




        // DEBUG
        trail.add(Main.robot.robotPos.getVector2D());
        GraphicSim.addDrawingGlobal(trail, Color.GREEN.darker());

        GraphicSim.updateOdometryDrawing(swerve.odo.robotPose);

        robotLinVel.putNumber("tarX", targetRobotSpeeds.x, Color.BLUE);
        robotLinVel.putNumber("tarY", targetRobotSpeeds.y, Color.BLUE);
        robotAngVel.putNumber("tarAng", targetRobotSpeeds.ang, Color.BLUE);

        robotLinVel.putNumber("curX", Main.robot.robotVel.x, Color.RED);
        robotLinVel.putNumber("curY", Main.robot.robotVel.y, Color.RED);
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