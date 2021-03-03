
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

        robotPose = robotPose.add(robotStep);

        robotPose.ang = Main.robot.robotPos.ang;
    }

    public void update(ArrayList<WheelData> arraylist){
        WheelData[] array = new WheelData[arraylist.size()];
        array = arraylist.toArray(array);
        update(array);
    }

    public static void main(String[] args) {
        OdometryExp odo = new OdometryExp(
            new Pose2D(+1, +1, +0),
            new Pose2D(-1, +1, +0),
            new Pose2D(-1, -1, +0),
            new Pose2D(+1, -1, +0)
        );

        odo.update(
            new WheelData(0, 1),
            new WheelData(0, 1),
            new WheelData(0, 1),
            new WheelData(0, 1)
        );
        System.out.println(odo.robotPose);

        odo.update(
            new WheelData(0, 1),
            new WheelData(0, 1),
            new WheelData(0, 1),
            new WheelData(0, 1)
        );
        System.out.println(odo.robotPose);

        odo.update(
            new WheelData(0, 1),
            new WheelData(0, 1),
            new WheelData(0, 1),
            new WheelData(0, 1)
        );
        System.out.println(odo.robotPose);

        odo.update(
            new WheelData(0, 1),
            new WheelData(0, 1),
            new WheelData(0, 1),
            new WheelData(0, 1)
        );
        System.out.println(odo.robotPose);

    }
}
