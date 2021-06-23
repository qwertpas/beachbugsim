package org.chis;

import org.chis.wrappers.AHRS;
import org.chis.wrappers.CANCoder;
import org.chis.wrappers.ControlMode;
import org.chis.wrappers.SPI.Port;
import org.chis.wrappers.SensorInitializationStrategy;
import org.chis.wrappers.TalonFXFeedbackDevice;
import org.chis.wrappers.WPI_TalonFX;

public class SwerveController {

    public Module[] modules;
    public AHRS gyro;

    public SwerveController(Module... modules){
        this.modules = modules;
        gyro = new AHRS(Port.kMXP);
    }

    public void nyoom(Vector2D linVel, double angVel){
        for(Module module : modules){
            module.moveModule(linVel.rotate(-gyro.getYaw()), angVel);
        }
    }

    public static class Module{

        public Vector2D placement;
        public WPI_TalonFX turnMotor, driveMotor;
        public CANCoder encoder;

        public Module(double x, double y, int turnID, int driveID, int encoderID){
            placement = new Vector2D(x, y);

            turnMotor = new WPI_TalonFX(turnID);
            driveMotor = new WPI_TalonFX(driveID);
            encoder = new CANCoder(encoderID);

            encoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
            turnMotor.configRemoteFeedbackFilter(encoder, 0);
            turnMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.RemoteSensor0, 0, 0);
            driveMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);

            turnMotor.config_kP(0, 0.1);
            turnMotor.config_kI(0, 0.01);
            turnMotor.config_kD(0, 0.001);

            driveMotor.config_kP(0, 0.00002);
            driveMotor.config_kI(0, 0.00005);
            driveMotor.config_kD(0, 0.0);
            driveMotor.config_kF(0, 0.00005);
        }
        
        void moveModule(Vector2D robotLinVel, double robotAngVel){
            Vector2D moduleRotationVector = placement.multScalar(robotAngVel).rotate90();
            Vector2D moduleMovementVector = robotLinVel.addVector(moduleRotationVector);

            double angleToTurn = normalize(Math.toDegrees(moduleMovementVector.getAngle()) - turnMotor.getSelectedSensorPosition());

            turnMotor.set(ControlMode.Position, turnMotor.getSelectedSensorPosition() + angleToTurn);
            driveMotor.set(ControlMode.Velocity, speedToTicksPer100Millis(moduleMovementVector.getMagnitude()));
        }

        static double normalize(double angle){
            angle %= 360.0; // [0..360) if angle is positive, (-360..0] if negative
            if (angle > 180.0) // was positive
                return angle - 360.0; // was (180..360) => returning (-180..0)
            if (angle <= -180.0) // was negative
                return angle + 360.0; // was (-360..180] => returning (0..180]
            return angle; // (-180..180]
        } 

        double speedToTicksPer100Millis(double speed) {
            double WHEEL_RADIUS = 0.0508;
            double DRIVE_RATIO = 6.86;
            double TICKS_PER_REV = 2048.0;
            return speed / WHEEL_RADIUS / (2 * Math.PI) * DRIVE_RATIO * TICKS_PER_REV / 10.0;
        }
    }
}