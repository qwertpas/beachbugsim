package org.chis;

import java.awt.Color;

import org.chis.sim.Constants;
import org.chis.sim.GraphicDash;
import org.chis.sim.Main;
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


public class UserCode{

    //KNOWN VARIABLES (METRIC BASE UNITS)
    static final double distBetweenWheelsX = 0.6096; //front to back
    static final double distBetweenWheelsY = 0.5588; //left to right
    static final double mass = 45.0;
    static final double wheelRadius = 0.0508;

    //WPILIB DECLARATIONS
    static AHRS gyro;
    static WPI_TalonFX fl_turn, fl_drive, bl_turn, bl_drive, br_turn, br_drive, fr_turn, fr_drive;
    static CANCoder fl_encoder, bl_encoder, br_encoder, fr_encoder;
    static Joystick joystick;

    //CUSTOM GRAPHS FOR DEBUG PURPOSES
    static GraphicDash demoGraph = new GraphicDash("Demo Title", 100, true);


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

        joystick = new Joystick(0);

        fl_turn.config_kP(0, 0.1);
        fl_turn.config_kI(0, 0.01);
        fl_turn.config_kD(0, 0.001);
    }

    public static void teleopPeriodic(){

        //The simulation will read your mouse position XY position if it can't find a USB joystick
        double x = joystick.getX();
        double y = joystick.getY();

        // fl_drive.set(ControlMode.PercentOutput, x);
        // bl_drive.set(ControlMode.PercentOutput, x);
        // br_drive.set(ControlMode.PercentOutput, x);
        // fr_drive.set(ControlMode.PercentOutput, x);

        double target = y*360;
        fl_turn.set(ControlMode.Position, target);
        // fr_turn.set(ControlMode.PercentOutput, y);




        // DEBUG ///////////////////////////////////////////////////////////////////////
        demoGraph.putNumber("fl_angle", fl_turn.getSelectedSensorPosition(), Color.BLUE);
        demoGraph.putNumber("fl_target", target, Color.RED);
        Printouts.put("fl angle", fl_turn.getSelectedSensorPosition());
        Printouts.put("Elapsed Time", Main.getElapsedTime());
    }

    public static double encoderToDist(double encoder){
        double driveGearRatio = 6.86;
        return encoder / MotorType.FALCON.TICKS_PER_REV / driveGearRatio * 2 * Math.PI * Constants.WHEEL_RADIUS.getDouble();
    }





}