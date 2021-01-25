package org.chis.sim;

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

        GraphicSim.init();
        Controls.init();
        Constants.calcConstants();

        new GraphicInput().setVisible(true);

        new UserCodeThread();

        startTime = System.nanoTime();
        while (true) {

            if(!paused){
                elaspedTime = (System.nanoTime() - GraphicInput.totalTimePaused - startTime) * 1e-9;
                robot.update(0.05);
                GraphicSim.sim.repaint();                
            }

            try {
                Thread.sleep(5);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }

    public static class UserCodeThread implements Runnable{
        private boolean exit;
        Thread t;
        UserCodeThread() {
            t = new Thread(this, "usercode");
            System.out.println("New Thread: " + t);
            exit = false;
            t.start();
        }

        public void run(){
            UserCode.initialize();
            Printouts printouts = new Printouts();

            while(!exit) {
                if(!paused){
                    GraphicSim.clearDrawing();
                    UserCode.execute();
                    Controls.updateControls();
                    printouts.repaint();
                    GraphicDash.paintAll();
                }
                try{
                    Thread.sleep(20);
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