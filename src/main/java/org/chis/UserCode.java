package org.chis;

import org.chis.SwerveController.Module;
import org.chis.sim.Printouts;
import org.chis.wrappers.Joystick;

public class UserCode{

    static final double distBetweenWheelsX = 0.6096; //front to back, meters
    static final double distBetweenWheelsY = 0.5588; //left to right, meters

    static Joystick joystick = new Joystick(0);

    static SwerveController swerve;

    public static void robotInit(){

        swerve = new SwerveController(
            new Module(
                +distBetweenWheelsX / 2.0, +distBetweenWheelsY / 2.0,
                1, 2, 9
            ),
            new Module(
                -distBetweenWheelsX / 2.0, +distBetweenWheelsY / 2.0,
                3, 4, 10
            ),
            new Module(
                -distBetweenWheelsX / 2.0, -distBetweenWheelsY / 2.0,
                5, 6, 11
            ),
            new Module(
                +distBetweenWheelsX / 2.0, -distBetweenWheelsY / 2.0,
                7, 8, 12
            )
        );
        
    }

    public static void teleopPeriodic(){

        Vector2D targetLinVel = new Vector2D(joystick.getX(), -joystick.getY()).multScalar(3);
        swerve.nyoom(targetLinVel, joystick.getZ() * -2);

        // GRAPHS AND PRINT OUTS ///////////////////////////////////////////////////////////
        Printouts.put("Elapsed Time", Main.getElapsedTime());
    }
}