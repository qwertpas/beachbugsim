package org.chis.sim.userclasses;

import java.awt.Color;
import java.util.ArrayList;

import org.chis.sim.*;
import org.chis.sim.math.*;
import org.chis.sim.wheels.CoaxSwerveModule;


public class UserCode{

    static ArrayList<Vector2D> trail = new ArrayList<Vector2D>();

    static GraphicDash dynamics = new GraphicDash("dynamics", 100, true);
    static GraphicDash state = new GraphicDash("state", 100, true);

    public static void initialize(){ //this function is run once when the robot starts
    }

    public static void execute(){ //this function is run 50 times a second (every 0.02 second)

        CoaxSwerveModule FL = (CoaxSwerveModule) Main.robot.wheels[0];
        CoaxSwerveModule BL = (CoaxSwerveModule) Main.robot.wheels[1];
        CoaxSwerveModule BR = (CoaxSwerveModule) Main.robot.wheels[2];
        CoaxSwerveModule FR = (CoaxSwerveModule) Main.robot.wheels[3];

        // double powerL = -Controls.rawY + Controls.rawX * 1;
        // double powerR = -Controls.rawY - Controls.rawX * 1;
        double powerL = 0.5;
        double powerR = 0.5;

        FL.driveMotor.setPower(powerL);
        BL.driveMotor.setPower(powerL);
        BR.driveMotor.setPower(powerR);
        FR.driveMotor.setPower(powerR);

        double turnpower = -Controls.rawY * 0.5;

        FL.turnMotor.setPower(turnpower);
        BL.turnMotor.setPower(turnpower);
        BR.turnMotor.setPower(turnpower);
        FR.turnMotor.setPower(turnpower);
        

        trail.add(Main.robot.robotPos.getVector2D());
        GraphicSim.addDrawingGlobal(trail, Color.GREEN.darker());

        //printing values in separate window
        dynamics.putNumber("netForce.x", Main.robot.netForce.x, Color.RED);
        dynamics.putNumber("netForce.y", Main.robot.netForce.y, Color.BLUE);
        dynamics.putNumber("netTorque", Main.robot.netTorque, Color.GREEN.darker());

        state.putNumber("pos", Main.robot.robotPos.getMagnitude(), Color.RED);
        state.putNumber("vel", Main.robot.robotVel.getMagnitude(), Color.BLUE);
        state.putNumber("acc", Main.robot.robotAcc.getMagnitude(), Color.GREEN.darker());

        Printouts.put("x", Main.robot.robotPos.x);
        Printouts.put("y", Main.robot.robotPos.y);
        Printouts.put("Heading", Main.robot.robotPos.ang);

        Printouts.put("FL angle", FL.wheelTurnIntegrator.pos);


    

    }




}