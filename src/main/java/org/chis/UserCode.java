package org.chis;

import java.awt.Color;

import org.chis.sim.Constants;
import org.chis.sim.GraphicDash;
import org.chis.sim.Motor.MotorType;
import org.chis.sim.Printouts;
import org.chis.wrappers.AHRS;
import org.chis.wrappers.CANCoder;
import org.chis.wrappers.ControlMode;
import org.chis.wrappers.Joystick;
import org.chis.wrappers.SPI;
import org.chis.wrappers.SensorInitializationStrategy;
import org.chis.wrappers.TalonFXFeedbackDevice;
import org.chis.wrappers.WPI_TalonFX;;

/**
 * Write and reference code here to control the simulated robot!
 * 
 * The example here initializes the hardware, makes the front left module spin towards an
 * angle determined by the joystick X axis, the back left motor drive at a constant 30%,
 * and the back right motor to try reach a velocity of 1000 encoder ticks per 100 ms. 
 */
public class UserCode{

    //KNOWN VARIABLES
    static final double distBetweenWheelsX = 0.6096; //front to back, meters
    static final double distBetweenWheelsY = 0.5588; //left to right, meters
    static final double mass = 45.0; //kilograms
    static final double wheelRadius = 0.0508; //meters

    //WPILIB DECLARATIONS
    static AHRS gyro;
    static WPI_TalonFX fl_turn, fl_drive, bl_turn, bl_drive, br_turn, br_drive, fr_turn, fr_drive;
    static CANCoder fl_encoder, bl_encoder, br_encoder, fr_encoder;

    //JOYSTICK
    /**
     * Port 0 will search for USB joysticks. If none are found, it will use your mouse cursor coordinates.
     * 
     * Port 1 will listen for data sent with the Syntien smartphone app. To get the ip of your computer,
     * run Main.java and look at the title of the simulation window. Use port 6036. Create a new interface 
     * named "joystick", then insert one 2dslider and two sliders. The 2dslider simulates X and Y axes,
     * the first slider simulates joystick throttle, and the second slider simulates the Z axis.
    */
    static Joystick joystick = new Joystick(0);

    //CUSTOM GRAPHS FOR DEBUG PURPOSES
    /** This creates a new window and you can plot points on it with putNumber() */
    static GraphicDash fl_angle = new GraphicDash("Front Left Angle", 100, true);
    static GraphicDash br_velo = new GraphicDash("Back Right Velocity", 100, true);
    static GraphicDash tr_angle = new GraphicDash("Target Angle", 100, true);


    public static void robotInit(){

        gyro = new AHRS(SPI.Port.kMXP);

        //FRONT LEFT MODULE
        fl_turn = new WPI_TalonFX(1);
        fl_drive = new WPI_TalonFX(2);
        fl_encoder = new CANCoder(9);

        //BACK LEFT MODULE
        bl_turn = new WPI_TalonFX(3);
        bl_drive = new WPI_TalonFX(4);
        bl_encoder = new CANCoder(10);

        //BACK RIGHT MODULE
        br_turn = new WPI_TalonFX(5);
        br_drive = new WPI_TalonFX(6);
        br_encoder = new CANCoder(11);

        //FRONT RIGHT MODULE
        fr_turn = new WPI_TalonFX(7);
        fr_drive = new WPI_TalonFX(8);
        fr_encoder = new CANCoder(12);

        //CONFIGURING ENCODERS FOR EACH MODULE
        fl_encoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
        fl_turn.configRemoteFeedbackFilter(fl_encoder, 0);
        fl_turn.configSelectedFeedbackSensor(TalonFXFeedbackDevice.RemoteSensor0, 0, 0);
        fl_drive.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);

        bl_encoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
        bl_turn.configRemoteFeedbackFilter(bl_encoder, 0);
        bl_turn.configSelectedFeedbackSensor(TalonFXFeedbackDevice.RemoteSensor0, 0, 0);
        bl_drive.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);

        br_encoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
        br_turn.configRemoteFeedbackFilter(br_encoder, 0);
        br_turn.configSelectedFeedbackSensor(TalonFXFeedbackDevice.RemoteSensor0, 0, 0);
        br_drive.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);

        fr_encoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
        fr_turn.configRemoteFeedbackFilter(fr_encoder, 0);
        fr_turn.configSelectedFeedbackSensor(TalonFXFeedbackDevice.RemoteSensor0, 0, 0);
        fr_drive.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);

        //EXAMPLE PID FOR FRONT LEFT MODULE ANGLE
        fl_turn.config_kP(0, 0.1);
        fl_turn.config_kI(0, 0.01);
        fl_turn.config_kD(0, 0.001);

        fr_turn.config_kP(0, 0.1);
        fr_turn.config_kI(0, 0.01);
        fr_turn.config_kD(0, 0.001);

        bl_turn.config_kP(0, 0.1);
        bl_turn.config_kI(0, 0.01);
        bl_turn.config_kD(0, 0.001);

        br_turn.config_kP(0, 0.1);
        br_turn.config_kI(0, 0.01);
        br_turn.config_kD(0, 0.001);

        //EXAMPLE PID FOR BACK RIGHT WHEEL VELOCITY
        fl_drive.config_kP(0, 0.001);
        fl_drive.config_kI(0, 0.001);
        fl_drive.config_kD(0, 0.0);
        fl_drive.config_kF(0, 0.00005);

        fr_drive.config_kP(0, 0.001);
        fr_drive.config_kI(0, 0.001);
        fr_drive.config_kD(0, 0.0);
        fr_drive.config_kF(0, 0.00005);

        bl_drive.config_kP(0, 0.001);
        bl_drive.config_kI(0, 0.001);
        bl_drive.config_kD(0, 0.0);
        bl_drive.config_kF(0, 0.00005);

        br_drive.config_kP(0, 0.001);
        br_drive.config_kI(0, 0.001);
        br_drive.config_kD(0, 0.0);
        br_drive.config_kF(0, 0.00005);
    }


    public static void teleopPeriodic(){

        double x = joystick.getX();
        double y = joystick.getY();

        double prevAngle = fl_turn.getSelectedSensorPosition() % 360;
        double angle = Math.atan2(y*Math.sqrt(1-0.5*x*x), x*Math.sqrt(1-0.5*y*y));
        double targetAngle = (Math.toDegrees(angle) + 360) % 360;

        double distance1 = Math.abs(prevAngle - targetAngle);
        double distance2 = (360 - distance1);

        double newAngle;
        if (distance1 <= distance2)
            newAngle = prevAngle - distance1;
        else
            newAngle = prevAngle + distance2;

        fl_turn.set(ControlMode.Position, newAngle);
        fr_turn.set(ControlMode.Position, newAngle);
        bl_turn.set(ControlMode.Position, newAngle);
        br_turn.set(ControlMode.Position, newAngle);

        double power = Math.sqrt(x*x + y*y);

        fl_drive.set(ControlMode.PercentOutput, power);
        fr_drive.set(ControlMode.PercentOutput, power);
        bl_drive.set(ControlMode.PercentOutput, power);
        br_drive.set(ControlMode.PercentOutput, power);

        // Simple set percent power 
        
        // // Running built-in PID to reach a certain angle
        // double targetAngle = x * 360; //in degrees
        // fl_turn.set(ControlMode.Position, targetAngle);

        // Running built-in PID to reach a certain velocity
        double targetVelocity = 1000; //in encoders ticks per 100ms
        // br_drive.set(ControlMode.Velocity, targetVelocity);


        // GRAPHS AND PRINT OUTS ///////////////////////////////////////////////////////////
        fl_angle.putNumber("fl_angle", fl_turn.getSelectedSensorPosition(), Color.BLUE);
        // fl_angle.putNumber("fl_targetAngle", targetAngle, Color.RED);
        tr_angle.putNumber("tr_angle", newAngle, Color.DARK_GRAY);


        br_velo.putNumber("br_velo", br_drive.getSelectedSensorVelocity(), Color.BLUE);
        br_velo.putNumber("br_targetVelo", targetVelocity, Color.RED);

        Printouts.put("heading", gyro.getYaw());
        Printouts.put("Elapsed Time", Main.getElapsedTime());
    }



    /**
     * @param encoder number of encoder ticks
     * @return distance a wheel has covered, in meters
     */
    public static double driveEncoderToDist(double encoder){
        double driveGearRatio = 6.86;
        return encoder / MotorType.FALCON.TICKS_PER_REV / driveGearRatio * 2 * Math.PI * Constants.WHEEL_RADIUS.getDouble();
    }
}