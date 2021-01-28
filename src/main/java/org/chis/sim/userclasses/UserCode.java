package org.chis.sim.userclasses;

import java.awt.Color;
import java.util.ArrayList;

import org.chis.sim.*;
import org.chis.sim.Motor.MotorType;
import org.chis.sim.math.*;
import org.chis.sim.wheels.CoaxSwerveModule;


public class UserCode{

    static ArrayList<Vector2D> trail = new ArrayList<Vector2D>();

    static GraphicDash dynamics = new GraphicDash("dynamics", 100, true);
    static GraphicDash state = new GraphicDash("state", 100, true);

    public static void initialize(){ //this function is run once when the robot starts
    }

    public static void execute(){ //this function is run 50 times a second (every 0.02 second)

        Motor FLdrive = Main.robot.motors.get(0);
        Motor FLturn = Main.robot.motors.get(1);

        Motor BLdrive = Main.robot.motors.get(2);
        Motor BLturn = Main.robot.motors.get(3);

        Motor BRdrive = Main.robot.motors.get(4);
        Motor BRturn = Main.robot.motors.get(5);

        Motor FRdrive = Main.robot.motors.get(6);
        Motor FRturn = Main.robot.motors.get(7);
        
        

        double powerL = -Controls.rawY + Controls.rawX * 1;
        double powerR = -Controls.rawY - Controls.rawX * 1;
        // double powerL = 0.5;
        // double powerR = 0.5;

        FLdrive.setPower(powerL);
        BLdrive.setPower(powerL);
        BRdrive.setPower(powerR);
        FRdrive.setPower(powerR);

        // double turnpower = -Controls.rawY * 0.5;
        double turnpower = -Controls.rawY * 0.0;

        FLturn.setPower(turnpower);
        BLturn.setPower(turnpower);
        BRturn.setPower(turnpower);
        FRturn.setPower(turnpower);
        

        trail.add(Main.robot.robotPos.getVector2D());
        GraphicSim.addDrawingGlobal(trail, Color.GREEN.darker());

        //printing values in separate window
        dynamics.putNumber("netForceMag", Main.robot.netForce.getMagnitude(), Color.RED);
        dynamics.putNumber("netTorque", Main.robot.netTorque, Color.GREEN.darker());

        state.putNumber("pos.x", Main.robot.robotPos.x, Color.BLACK);
        state.putNumber("FL pos", encoderToDist(FLdrive.getEncoderPosition()), Color.BLUE);
        state.putNumber("BL pos", encoderToDist(BLdrive.getEncoderPosition()), Color.GREEN);
        state.putNumber("BR pos", encoderToDist(BRdrive.getEncoderPosition()), Color.RED);
        state.putNumber("FR pos", encoderToDist(FRdrive.getEncoderPosition()), Color.ORANGE);

        Printouts.put("x", Main.robot.robotPos.x);
        Printouts.put("y", Main.robot.robotPos.y);
        Printouts.put("Heading", Main.robot.robotPos.ang);
    

    }

    public static double encoderToDist(double encoder){
        return encoder / MotorType.FALCON.TICKS_PER_REV / 6.86 * 2 * Math.PI * Constants.WHEEL_RADIUS.getDouble();
    }




}