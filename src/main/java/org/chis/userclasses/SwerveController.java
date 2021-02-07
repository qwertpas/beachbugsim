package org.chis.userclasses;

import org.chis.sim.Constants;
import org.chis.sim.Main;
import org.chis.sim.Util;
import org.chis.sim.Util.PID;
import org.chis.sim.math.Pose2D;
import org.chis.sim.math.Vector2D;
import org.chis.sim.math.Vector2D.Type;

public class SwerveController {

    Module[] modules;

    static final double TICKS_PER_REV = 2048;
    static final double TURN_RATIO = 12.8;
    static final double DRIVE_RATIO = 6.86;
    static final double WHEEL_RADIUS = Util.inchesToMeters(2);
    static final double DT = Constants.USERCODE_DT.getDouble();

    public SwerveController(Module... modules){
        this.modules = modules;
    }

    public void move(Pose2D robotSpeeds){
        for(Module module : modules){
            module.move(robotSpeeds);
        }
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

        public Module(int turnMotorID, int driveMotorID, Pose2D placement){
            this.turnMotorID = turnMotorID;
            this.driveMotorID = driveMotorID;
            this.placement = placement;

            turnPID.setkP(2);
            turnPID.setkI(15, 0.5, 0.5);
            turnPID.setkD(0.1);

            drivePID.setkP(0.1);
            drivePID.setkI(1, 1, 0.5);
            drivePID.setkD(0.001);
        }

        public void move(Pose2D robotSpeeds){
            Vector2D linVelo = robotSpeeds.getVector2D();
            double angVelo = robotSpeeds.ang;

            if(linVelo.getMagnitude() < 0.1 && Math.abs(angVelo) < 0.1){
                Main.robot.motors.get(turnMotorID).setPower(0);
                Main.robot.motors.get(driveMotorID).setPower(0);
                return;
            }

            // ask chis for vector math derivation
            targetSpeedVector = linVelo.add(placement.scalarMult(angVelo).rotate90());
            
            currentAngle = getAngle();
            currentDriveSpeed = getDriveSpeed();

            targetAngle = targetSpeedVector.getAngle();
            targetDriveSpeed = targetSpeedVector.getMagnitude();
            
            
            if(Math.abs(targetDriveSpeed) < 0.5){
                targetAngle = calcClosestModuleAngle(currentAngle, targetAngle);
                if(reversed) targetDriveSpeed *= -1;
            }else{
                if(reversed){
                    targetAngle = calcClosestModuleAngle360(currentAngle, targetAngle + Math.PI);
                    targetDriveSpeed *= -1;
                }else{
                    targetAngle = calcClosestModuleAngle360(currentAngle, targetAngle);
                }
            }

            // targetAngle = calcClosestModuleAngle360(currentAngle, targetAngle);

            turnPower = turnPID.loop(currentAngle, targetAngle, DT);
            drivePower = drivePID.loop(currentDriveSpeed, targetDriveSpeed, DT) + targetDriveSpeed * 0.23;

            Main.robot.motors.get(turnMotorID).setPower(turnPower);
            Main.robot.motors.get(driveMotorID).setPower(drivePower);
        }
        

        private double calcClosestModuleAngle(double currentAngle, double targetAngle){
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

        private double calcClosestModuleAngle360(double currentAngle, double targetAngle){
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
    }

    public static void main(String[] args) {
        for(int i = -720; i < 720; i += 10){
            System.out.println(i + " : " + Util.normalizeAngle(i, 180));
        }
    }
}
