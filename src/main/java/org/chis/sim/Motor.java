package org.chis.sim;

public class Motor{

    public enum MotorType{

        // ADD MOTORS HERE
        FALCON(6380, 4.69, 2048),
        NEO(5880, 3.36, 42),
        CIM(5330, 2.41, 0),
        MINICIM(5840, 1.41, 0),
        ;

        public double FREE_SPEED, STALL_TORQUE, TICKS_PER_REV; // rad/sec and newton-meters
        MotorType(double MAX_RPM, double STALL_TORQUE, double TICKS_PER_REV){
            this.STALL_TORQUE = STALL_TORQUE;
            this.FREE_SPEED = Util.rpmToRadSec(MAX_RPM);
            this.TICKS_PER_REV = TICKS_PER_REV;
        }
    }

    public final MotorType motorType;
    public double voltage;
    public double angVelo; //angular velocity in radians per second
    public double angVeloPrev; //store previous angular velocity 
    public double torque; //newton*meters
    public double position; //motor shaft in radians
    public double numMotors;

    public Motor(MotorType motorType, double numMotors){
        this.numMotors = numMotors;
        this.motorType = motorType;
    }

    public void setPower(double power){
        voltage = power * Constants.MAX_VOLTAGE.getDouble(); //assumes that robot system voltage is always at max (no brownout)
    }

    public void update(double radPerSec_input, double dt){
        angVelo = radPerSec_input;

        //calculates torque based on motor torque-angvelo graph: https://www.desmos.com/calculator/nmge6gksgj 
        torque = motorType.STALL_TORQUE * ((voltage / Constants.MAX_VOLTAGE.getDouble()) - (angVelo / motorType.FREE_SPEED));

        integrateEncoder();
    }

    //integrates angular velocity to get angular position 
    public long lastTime = System.nanoTime();
    public double dt;
    public void integrateEncoder(){
        dt = (System.nanoTime() - lastTime) * 1e-9; //change in time (seconds) used for integrating
        lastTime = System.nanoTime();

        position += 0.5 * (angVelo + angVeloPrev) * dt;
        angVeloPrev = angVelo;
    }

    public double getEncoderPosition(){
        double revolutions = position / (2 * Math.PI); //convert the position in radians to position in revolutions
        double encoderTicks = revolutions * motorType.TICKS_PER_REV; //convert revolutions to encoder ticks
        return encoderTicks;
    }

    public double getEncoderVelocity(){
        double rps = angVelo / (2 * Math.PI);
        double encoderTicksPerSec = rps * motorType.TICKS_PER_REV;
        return encoderTicksPerSec;
    }

    public void resetEncoder(){
        position = 0;
    }

    
}