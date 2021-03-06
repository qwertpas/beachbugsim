
package org.chis.userclasses;

import java.util.ArrayList;

import org.chis.sim.Main;
import org.chis.sim.math.Pose2D;
import org.ejml.simple.*;

public class OdometryExp {
    public Pose2D robotPose = new Pose2D();

    Pose2D[] placements;

    public OdometryExp(Pose2D ... placements){
        this.placements = placements;
    }

    public void update(WheelData ... wheelData){
        if(wheelData.length != placements.length){
            throw new RuntimeException("num wheels not same");
        }

        SimpleMatrix A = new SimpleMatrix(placements.length * 2, 3);
        SimpleMatrix y = new SimpleMatrix(placements.length * 2, 1);

        for(int wheelIndex = 0; wheelIndex < placements.length; wheelIndex++){
            double dist = wheelData[wheelIndex].dist;
            double angle = wheelData[wheelIndex].angle;
            // System.out.println("now filling: " + wheelIndex);
            A.setRow(2 * wheelIndex, 0, 1, 0, -placements[wheelIndex].y);
            A.setRow(2 * wheelIndex + 1, 0, 0, 1, placements[wheelIndex].x);
            y.setRow(2 * wheelIndex, 0, dist * Math.cos(angle));
            y.setRow(2 * wheelIndex + 1, 0, dist * Math.sin(angle));
        }

        SimpleMatrix x = A.solve(y);

        Pose2D robotStep = new Pose2D(x.get(0), x.get(1), x.get(2)).rotateVec(robotPose.ang);

        robotPose = robotPose.exp(robotStep);

        // robotPose.ang = Main.robot.robotPos.ang;
    }

    public void update(ArrayList<WheelData> arraylist){
        WheelData[] array = new WheelData[arraylist.size()];
        array = arraylist.toArray(array);
        update(array);
    }

    public static void main(String[] args) {
        Pose2D initPose = new Pose2D(0, 0, Math.toRadians(66.782));
        Pose2D step = new Pose2D(2.56, 0, -Math.toRadians(180 - 138.148));

        System.out.println("exp: " + initPose.exp(step));
        System.out.println("add: " + initPose.add(step));

    }
}
