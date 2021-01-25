package org.chis.sim.userclasses;

import java.awt.Color;
import java.util.ArrayList;

import org.chis.sim.*;
import org.chis.sim.math.*;
import org.chis.sim.wheels.CoaxSwerveModule;
import org.chis.sim.wheels.Wheel;


public class UserCode{

    static ArrayList<Vector2D> trail = new ArrayList<Vector2D>();

    static GraphicDash force = new GraphicDash("force", 100, true);
    static GraphicDash state = new GraphicDash("state", 100, true);

    public static void initialize(){ //this function is run once when the robot starts
    }

    public static void execute(){ //this function is run 50 times a second (every 0.02 second)

        for(Wheel wheel : Main.robot.wheels){
            var module = (CoaxSwerveModule) wheel;
            module.driveMotor.setPower(1);
        }
        

        trail.add(Main.robot.robotPos.getVector2D());
        GraphicSim.addDrawingGlobal(trail, Color.GREEN.darker());

        //printing values in separate window
        force.putNumber("force", Main.robot.netForce.x, Color.RED);
        // state.putNumber("pos", Main.robot.robotPos.x, Color.RED);
        state.putNumber("vel", Main.robot.robotVel.x, Color.BLUE);
        state.putNumber("acc", Main.robot.robotAcc.x, Color.GREEN.darker());

        Printouts.put("x", Main.robot.robotPos.x);
        Printouts.put("y", Main.robot.robotPos.y);
        Printouts.put("Heading", Main.robot.robotPos.ang);

    

    }




}