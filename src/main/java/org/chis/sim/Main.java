package org.chis.sim;

import java.awt.Color;
import java.util.ArrayList;

import org.chis.UserCode;
import org.chis.sim.Util.LooptimeMonitor;
import org.chis.sim.math.Vector2D;

//runs all of the components together
public class Main {

    public static Boolean paused = true;

    public static double startTime;
    public static double elaspedTime;

    public static Robot robot;

    public static void main(String[] args) {

        robot = new Robot();
        robot.init();

        new GraphicInput().setVisible(true);

        new UserCodeThread();
        new DisplayThread();

        startTime = System.nanoTime();
        while (true) {

            if(!paused){
                elaspedTime = (System.nanoTime() - GraphicInput.totalTimePaused - startTime) * 1e-9 * Constants.SIMSPEED.getDouble();
                robot.update(Constants.PHYSICS_DT.getDouble());
            }

            try {
                Thread.sleep((int) (1000 * Constants.PHYSICS_DT.getDouble() / Constants.SIMSPEED.getDouble()));
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }

    public static double getElapsedTime(){
        return elaspedTime;
    }

    public static class DisplayThread implements Runnable{
        private boolean exit;
        Thread t;
        DisplayThread() {
            t = new Thread(this, "Display");
            System.out.println("New Thread: " + t);
            exit = false;
            t.start();
        }

        public void run(){
            GraphicSim.init();

            LooptimeMonitor clock = new LooptimeMonitor();

            while(!exit) {
                if(!paused){
                    clock.start();

                    GraphicSim.sim.repaint();

                    clock.end();
                    // System.out.println("Display looptime: " + 1000 * clock.looptime);
                }
                try{
                    double sleeptime = Math.max(0, Constants.DISPLAY_DT.getDouble() / Constants.SIMSPEED.getDouble() - clock.codetime); //in seconds
                    Thread.sleep((int) (sleeptime * 1000)); //in milliseconds
                }catch(InterruptedException e){
                    e.printStackTrace();
                }
            }
        }
        public void stop(){
            exit = true;
        }
    }

    public static ArrayList<Vector2D> trail = new ArrayList<Vector2D>();
    static GraphicDash robotLinVel = new GraphicDash("Linear Velocity", 100, true);
    static GraphicDash robotAngVel = new GraphicDash("Angular Velocity", 100, true);

    public static class UserCodeThread implements Runnable{
        private boolean exit;
        Thread t;
        UserCodeThread() {
            t = new Thread(this, "Usercode");
            System.out.println("New Thread: " + t);
            exit = false;
            t.start();
        }

        public void run(){

            Controls.init();
            UserCode.robotInit();

            new Printouts();

            LooptimeMonitor clock = new LooptimeMonitor();

            while(!exit) {
                if(!paused){
                    clock.start();

                    robotLinVel.putNumber("Vx", Main.robot.robotVel.rotate(-Main.robot.robotPos.ang).x, Color.RED);
                    robotLinVel.putNumber("Vy", Main.robot.robotVel.rotate(-Main.robot.robotPos.ang).y, Color.GREEN.darker());
                    robotAngVel.putNumber("ω", Main.robot.robotVel.ang, Color.BLUE);

                    GraphicSim.clearDrawing();
                    UserCode.teleopPeriodic();
                    Controls.updateControls();
                    GraphicDash.paintAll();

                    trail.add(robot.robotPos);
                    if(trail.size() > 100){
                        trail.remove(0);
                    }
                    GraphicSim.addDrawingGlobal(trail, Color.RED);

                    clock.end();

                    // System.out.println("Usercode looptime: " + 1000 * clock.looptime);
                }

                try{
                    double sleeptime = Math.max(0, Constants.USERCODE_DT.getDouble() / Constants.SIMSPEED.getDouble() - clock.codetime); //in seconds
                    Thread.sleep((int) (sleeptime * 1000)); //in milliseconds
                }catch(InterruptedException e){
                    e.printStackTrace();
                }
            }
        }
        public void stop(){
            exit = true;
        }
    }



    
    

}