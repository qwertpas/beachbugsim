package org.chis.userclasses;

import java.util.ArrayList;

import org.chis.sim.Constants;
import org.chis.sim.Main;
import org.chis.sim.Util;
import org.chis.sim.Util.PID;
import org.chis.sim.math.Pose2D;
import org.chis.sim.math.Vector2D;
import org.chis.sim.math.Vector2D.Type;

public class SwerveController {

    public Module[] modules;
    public OdometryExp odo;
    public Gyro gyro;

    public Pose2D targetSpeeds = new Pose2D();

    public static final double TICKS_PER_REV = 2048;
    public static final double TURN_RATIO = 12.8;
    public static final double DRIVE_RATIO = 6.86;
    public static final double WHEEL_RADIUS = Util.inchesToMeters(2);
    public static final double DT = Constants.USERCODE_DT.getDouble();

    public SwerveController(Gyro gyro, Module... modules){
        this.gyro = gyro;
        this.modules = modules;

        Pose2D[] placements = new Pose2D[modules.length];
        for(int moduleIndex = 0; moduleIndex < modules.length; moduleIndex++){
            placements[moduleIndex] = modules[moduleIndex].placement;
        }

        odo = new OdometryExp(gyro, placements);
    }

    public void init(Pose2D initPose){
        for(Module module : modules){
            module.lastDrivePos = 0;
            module.currentDrivePos = 0;
        }
        odo.setPose(initPose);
    }

    public void nyoom(Pose2D robotSpeeds){
        this.targetSpeeds = robotSpeeds;

        ArrayList<WheelData> wheelSteps = new ArrayList<WheelData>();
        for(Module module : modules){
            module.move(robotSpeeds);

            double avgAngle = (module.currentAngle + module.lastAngle) / 2.0;
            double distStep = module.currentDrivePos - module.lastDrivePos;
            wheelSteps.add(new WheelData(avgAngle, distStep));
        }

        odo.update(wheelSteps);
    }

    public void nyoom(Vector2D robotSpeeds){
        nyoom(new Pose2D(robotSpeeds, 0));
    }

    public void nyoomToPoint(Vector2D endpointGlobal, double speed){
        Vector2D endpointRel = odo.robotPose.getRelative(endpointGlobal);
        nyoom(new Vector2D(speed, endpointRel.getAngle(), Type.POLAR));
    }

    public void nyoomAboutPoint(Vector2D centerGlobal, double speed, boolean rotate){
        Vector2D centerRel = odo.robotPose.getRelative(centerGlobal);
        // Printouts.put("centerGlobal", centerGlobal);
        // Printouts.put("centerRel", centerRel);
        double angVel = speed / centerRel.getMagnitude();

        if(rotate){
            nyoom(new Pose2D(centerRel.scalarMult(-angVel).rotate90(), angVel));
        }else{
            nyoom(new Pose2D(centerRel.scalarMult(-angVel).rotate90(), 0));
        }
    }

    public void nyoomToPointAndSpin(Vector2D endpointGlobal, double angVel, double speed){
        Vector2D endpointRel = odo.robotPose.getRelative(endpointGlobal);
        nyoom(new Pose2D(new Vector2D(speed, endpointRel.getAngle(), Type.POLAR), angVel));
    }

    public void stop(){
        nyoom(new Pose2D());
    }


    public static class Module{
        Pose2D placement;
        int turnMotorID, driveMotorID;

        Vector2D targetSpeedVector;
        double targetAngle, targetDriveSpeed;
        boolean reversed;

        double currentAngle, currentDriveSpeed;

        PID turnPID = new PID();
        PID drivePID = new PID();

        double turnPower, drivePower;

        //stuff for odometry:
        double lastAngle, lastDrivePos, currentDrivePos;

        public Module(int turnMotorID, int driveMotorID, Pose2D placement){
            this.turnMotorID = turnMotorID;
            this.driveMotorID = driveMotorID;
            this.placement = placement;

            turnPID.setkP(4);
            turnPID.setkI(15, 0.5, 0.5);
            turnPID.setkD(0.1);

            drivePID.setkP(0.1);
            drivePID.setkI(1, 1, 0.5);
            drivePID.setkD(0.001);
        }

        public void move(Pose2D robotSpeeds){
            Vector2D linVelo = robotSpeeds.getVector2D();
            double angVelo = robotSpeeds.ang;

            // ask chis for vector math derivation
            targetSpeedVector = linVelo.add(placement.scalarMult(angVelo).rotate90());

            lastAngle = currentAngle;
            currentAngle = getAngle();
            lastDrivePos = currentDrivePos;
            currentDrivePos = getDrivePos();
            currentDriveSpeed = getDriveSpeed();


            targetAngle = targetSpeedVector.getAngle();
            targetDriveSpeed = targetSpeedVector.getMagnitude();
            
            
            if(Math.abs(targetDriveSpeed) < 10){ // was 0.5
                targetAngle = calcClosestModuleAngle180(currentAngle, targetAngle);
                if(reversed) targetDriveSpeed *= -1;
            }else{
                if(reversed){
                    targetAngle = calcClosestModuleAngle360(currentAngle, targetAngle + Math.PI);
                    targetDriveSpeed *= -1;
                }else{
                    targetAngle = calcClosestModuleAngle360(currentAngle, targetAngle);
                }
            }

            //deadband
            if(linVelo.getMagnitude() < 0.1 && Math.abs(angVelo) < 0.1){
                linVelo = new Vector2D();
                angVelo = 0;

                Main.robot.motors.get(turnMotorID).setPower(0);
                Main.robot.motors.get(driveMotorID).setPower(0);
            }else{
                turnPower = turnPID.loop(currentAngle, targetAngle, DT);
                drivePower = drivePID.loop(currentDriveSpeed, targetDriveSpeed, DT) + targetDriveSpeed * 0.23;
    
                Main.robot.motors.get(turnMotorID).setPower(turnPower);
                Main.robot.motors.get(driveMotorID).setPower(drivePower);
            }


        }
        

        public double calcClosestModuleAngle180(double currentAngle, double targetAngle){
            double differencePi = (currentAngle - targetAngle) % Math.PI; //angle error from (-180, 180)
    
            double closestAngle;
            if(Math.abs(differencePi) < (Math.PI / 2.0)){ //chooses closer of the two acceptable angles closest to currentAngle
                closestAngle = currentAngle - differencePi;
            }else{
                closestAngle = currentAngle - differencePi + Math.copySign(Math.PI, differencePi);
            }
    
            double difference2Pi = (closestAngle - targetAngle) % (2 * Math.PI);
            reversed = Math.abs(difference2Pi) > (Math.PI / 2.0); //if the difference is closer to 180, reverse direction 
    
            return closestAngle;
        }

        public double calcClosestModuleAngle360(double currentAngle, double targetAngle){
            double diffNormalized = Util.normalizeAngle(currentAngle - targetAngle, Math.PI); //angle error from (-PI, PI)
            return currentAngle - diffNormalized;
        }

        private double getAngle(){
            double encoderPos = Main.robot.motors.get(turnMotorID).getEncoderPosition();
            return encoderPos / TICKS_PER_REV / TURN_RATIO * 2 * Math.PI + placement.ang;
        }

        private double getDriveSpeed(){
            double encoderVel = Main.robot.motors.get(driveMotorID).getEncoderVelocity();
            return encoderVel / TICKS_PER_REV / DRIVE_RATIO * 2 * Math.PI * WHEEL_RADIUS;
        }

        private double getDrivePos(){
            double encoderPos = Main.robot.motors.get(driveMotorID).getEncoderPosition();
            return encoderPos / TICKS_PER_REV / DRIVE_RATIO * 2 * Math.PI * WHEEL_RADIUS;
        }
    }
}
