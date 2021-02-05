package org.chis.userclasses;

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

        double targetAngle, targetAngleOptimized, targetDriveSpeed;
        boolean reversed;

        double currentAngle, currentDriveSpeed;

        PID turnPID = new PID();
        PID drivePID = new PID();

        double turnPower, drivePower;

        public Module(int turnMotorID, int driveMotorID, Pose2D placement){
            this.turnMotorID = turnMotorID;
            this.driveMotorID = driveMotorID;
            this.placement = placement;

            turnPID.setkP(3);
            turnPID.setkI(4, 0.5, 0.1);

            drivePID.setkP(0.1);
            drivePID.setkI(2, 1, 1);
        }

        public void move(Pose2D robotSpeeds){
            Vector2D linVelo = robotSpeeds.getVector2D();
            double angVelo = robotSpeeds.ang;

            // ask chis for vector math derivation
            targetAngle = linVelo.add(placement.rotate90().scalarMult(angVelo)).getAngle();
            targetDriveSpeed = linVelo.getMagnitude() + placement.getMagnitude() * angVelo;
            
            currentAngle = getAngle();
            currentDriveSpeed = getDriveSpeed();

            targetAngleOptimized = calcClosestModuleAngle(currentAngle, targetAngle);
            if(reversed) targetDriveSpeed *= -1;

            turnPower = turnPID.loop(currentAngle, targetAngleOptimized);
            drivePower = drivePID.loop(currentDriveSpeed, targetDriveSpeed) + targetDriveSpeed * 0.45;

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

        private double getAngle(){
            double encoderPos = Main.robot.motors.get(turnMotorID).getEncoderPosition();
            return encoderPos / TICKS_PER_REV / TURN_RATIO * 2 * Math.PI;
        }

        private double getDriveSpeed(){
            double encoderVel = Main.robot.motors.get(driveMotorID).getEncoderVelocity();
            return encoderVel / TICKS_PER_REV / DRIVE_RATIO * 2 * Math.PI * WHEEL_RADIUS;
        }
    }

    public static void main(String[] args) {

        double vx = 2;
        double vy = 0;

        double px = -2;
        double py = -1;

        double angVelo = 0.1;

        double wheelangleX = vx - py * angVelo;
        double wheelangleY = vy + px * angVelo;

        double angle = Math.atan2(wheelangleY, wheelangleX);

        Vector2D placement = new Vector2D(-2, -1, Type.CARTESIAN);
        Vector2D velo = new Vector2D(2, 0, Type.CARTESIAN);
        double angVec = velo.add(placement.rotate90().scalarMult(angVelo)).getAngle();

        System.out.println(angle);
        System.out.println(angVec);
    }
}
