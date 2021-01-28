package org.chis.sim;

import org.chis.sim.Util.LooptimeMonitor;
import org.chis.sim.userclasses.UserCode;

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
                elaspedTime = (System.nanoTime() - GraphicInput.totalTimePaused - startTime) * 1e-9;
                robot.update(Constants.PHYSICS_DT.getDouble());
            }

            try {
                Thread.sleep((int) (1000 * Constants.PHYSICS_DT.getDouble() / Constants.SIMSPEED.getDouble()));
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
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
                    double sleeptime = Math.max(0, Constants.DISPLAY_DT.getDouble() - clock.codetime); //in seconds
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
            UserCode.initialize();

            Printouts printouts = new Printouts();

            LooptimeMonitor clock = new LooptimeMonitor();

            while(!exit) {
                if(!paused){
                    clock.start();

                    GraphicSim.clearDrawing();
                    UserCode.execute();
                    Controls.updateControls();
                    printouts.repaint();
                    GraphicDash.paintAll();

                    clock.end();

                    // System.out.println("Usercode looptime: " + 1000 * clock.looptime);
                }

                try{
                    double sleeptime = Math.max(0, Constants.USERCODE_DT.getDouble() - clock.codetime); //in seconds
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