package org.chis.wrappers;

import org.chis.Main;
import org.chis.sim.Motor;
import org.chis.sim.Util.PID;

public class WPI_TalonFX {

    private Motor motor;
    private CANCoder cancoder0 = null;
    private CANCoder cancoder1 = null;
    private int whichEncoder = 2;

    private PID pid = new PID();
    private double kF = 0;

    public WPI_TalonFX(int deviceNumber){
        if(deviceNumber == 0){
            System.out.println("CAN Device 0 not found");
            System.exit(0);
        }
        int motorNumber = deviceNumber;
        if(deviceNumber % 2 == 0){
            motorNumber -= 2;
        }

        motor = Main.robot.motors.get(motorNumber);
    }


    public void set(ControlMode mode, double value){
        if(mode == ControlMode.PercentOutput){
            motor.setPower(value);
        }

        if(mode == ControlMode.Position){
            pid.loop(getSelectedSensorPosition(), value, 0.02);
            motor.setPower(pid.getPower() + kF*value);
        }

        if(mode == ControlMode.Velocity){
            pid.loop(getSelectedSensorVelocity(), value, 0.02); //ticks per 100ms
            motor.setPower(pid.getPower() + kF*value);
        }
    }

    public void configRemoteFeedbackFilter(CANCoder canCoderRef, int remoteOrdinal){
        if(remoteOrdinal == 0){
            cancoder0 = canCoderRef;
        }
        else if(remoteOrdinal == 1){
            cancoder1 = canCoderRef;
        }
        else{
            System.out.println("Remote ordinal must be 0 or 1");
            System.exit(0);
        }
    }

    public void configSelectedFeedbackSensor(TalonFXFeedbackDevice feedbackDevice, int pidIdx, int timeoutMs){
        if(feedbackDevice == TalonFXFeedbackDevice.RemoteSensor0){
            whichEncoder = 0;
        }else if(feedbackDevice == TalonFXFeedbackDevice.RemoteSensor1){
            whichEncoder = 1;
        }else{
            whichEncoder = 2;
        }
    }

    /** @return encoder ticks */
    public double getSelectedSensorPosition(){
        if(whichEncoder == 0){
            return cancoder0.getPosition();
        }else if(whichEncoder == 1){
            return cancoder1.getPosition();
        }else{
            return motor.getEncoderPosition();
        }
    }

    /** @return encoder ticks per 100ms */
    public double getSelectedSensorVelocity(){
        if(whichEncoder == 0){
            return cancoder0.getVelocity();
        }else if(whichEncoder == 1){
            return cancoder1.getVelocity();
        }else{
            return motor.getEncoderVelocity() * 0.1; //encoder ticks per 100ms
        }
    }

    public void config_kP(int slotIdx, double value){
        if(slotIdx != 0){
            System.out.println("Use slotIDx 0 when configuring PID.");
            System.exit(0);
        }
        pid.setkP(value);
    }

    public void config_kI(int slotIdx, double value){
        if(slotIdx != 0){
            System.out.println("Use slotIDx 0 when configuring PID.");
            System.exit(0);
        }
        pid.setkI(value, Double.MAX_VALUE, 1);
    }

    public void config_kD(int slotIdx, double value){
        if(slotIdx != 0){
            System.out.println("Use slotIDx 0 when configuring PID.");
            System.exit(0);
        }
        pid.setkD(value);
    }

    public void config_kF(int slotIdx, double value){
        if(slotIdx != 0){
            System.out.println("Use slotIDx 0 when configuring PID.");
            System.exit(0);
        }
        kF = value;
    }
}
