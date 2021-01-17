package org.chis.sim.userclasses;

import java.awt.Color;
import java.util.ArrayList;

import org.chis.sim.*;
import org.chis.sim.Util.PID;
import org.chis.sim.Util.Vector2D;
import org.chis.sim.Util.Vector2D.Type;
import org.chis.sim.userclasses.paths.PurePursuit;


public class UserCode{

    static GraphicDash leftVelo = new GraphicDash("leftVelo", 100, true);
    static GraphicDash rightVelo = new GraphicDash("rightVelo", 100, true);

    static PurePursuit follower;
    static ArrayList<Vector2D> path = new ArrayList<Vector2D>();
    static ArrayList<Vector2D> trail = new ArrayList<Vector2D>();
    static PID leftPID = new PID();
    static PID rightPID = new PID();

    public static void initialize(){ //this function is run once when the robot starts

        // path.add(new Vector2D(-6, -2, Type.CARTESIAN));

        path.add(new Vector2D(-1, 1, Type.CARTESIAN));

        path.add(new Vector2D(2, -5, Type.CARTESIAN));
        path.add(new Vector2D(-2, -5, Type.CARTESIAN));
        path.add(new Vector2D(3, -2, Type.CARTESIAN));

        follower = new PurePursuit(path, 0.75, Constants.HALF_DIST_BETWEEN_WHEELS, 3);

        leftPID.setkP(2);
        leftPID.setkI(3, 1, 0.5);
        leftPID.setkD(0.0);
        rightPID.copyConstants(leftPID);
    }

    public static void execute(){ //this function is run 50 times a second (every 0.02 second)

        double[] velos = follower.getVelocities(Main.robot.getPos(), Main.robot.heading, 2);
        double targetvL = velos[0];
        double targetvR = velos[1];

        double currentvL = convertEncoderΤοDist(Main.robot.leftEncoderVelocity());
        double currentvR = convertEncoderΤοDist(Main.robot.rightEncoderVelocity());
        leftPID.loop(currentvL, targetvL);
        rightPID.loop(currentvR, targetvR);

        double leftPower = leftPID.getPower() + targetvL * 0.3;
        double rightPower = rightPID.getPower() + targetvR * 0.3;
        
        Main.robot.leftGearbox.setPower(leftPower);
        Main.robot.rightGearbox.setPower(rightPower);

        GraphicSim.addDrawingGlobal(path, Color.BLACK);
        trail.add(Main.robot.getPos());
        GraphicSim.addDrawingGlobal(trail, Color.GREEN.darker());

        //printing values in separate window
        Printouts.put("x", Main.robot.x);
        Printouts.put("y", Main.robot.y);
        Printouts.put("Heading", Main.robot.heading);
        Printouts.put("curvature", follower.curvature);
        Printouts.put("distRemaining", follower.distRemaining);
        leftVelo.putNumber("targetvL", targetvL, Color.BLUE);
        rightVelo.putNumber("targetvR", targetvR, Color.BLUE);
        leftVelo.putNumber("currentvL", convertEncoderΤοDist(Main.robot.leftEncoderVelocity()), Color.RED);
        rightVelo.putNumber("currentvR", convertEncoderΤοDist(Main.robot.rightEncoderVelocity()), Color.RED);

    }



    
    private static double convertEncoderΤοDist(double encoder){
        return encoder / Constants.TICKS_PER_REV.getDouble() / Constants.GEAR_RATIO.getDouble() * 2 * Math.PI * Constants.WHEEL_RADIUS.getDouble();
    } 


    



}