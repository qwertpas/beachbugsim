
package org.chis.userclasses;

import java.util.ArrayList;

import org.chis.sim.math.Pose2D;
import org.chis.sim.math.Vector2D;
import org.chis.sim.math.Vector2D.Type;

public class OdometryAvg {
    public Pose2D robotPose = new Pose2D();

    Gyro gyro;
    Pose2D[] placements;

    double lastAngle;

    public OdometryAvg(Gyro gyro, Pose2D ... placements){
        this.gyro = gyro;
        this.placements = placements;
    }

    public void setPose(Pose2D newPose){
        robotPose = newPose;
    }

    public void update(WheelData ... wheelData){
        if(wheelData.length != placements.length){
            throw new RuntimeException("num wheels not same");
        }

        Vector2D robotStep = new Vector2D();
        for(WheelData wheel : wheelData){
            robotStep = robotStep.add(new Vector2D(wheel.dist, wheel.angle, Type.POLAR));
        }
        robotStep = robotStep.scalarDiv(wheelData.length).rotate(0.5*(gyro.getContinuousAngle() - lastAngle));

        robotPose = robotPose.exp(new Pose2D(robotStep, gyro.getContinuousAngle() - lastAngle));
        robotPose.ang = gyro.getContinuousAngle();
        lastAngle = gyro.getContinuousAngle();
    }

    public void update(ArrayList<WheelData> arraylist){
        WheelData[] array = new WheelData[arraylist.size()];
        array = arraylist.toArray(array);
        update(array);
    }

    public static void main(String[] args) {
        Pose2D initPose = new Pose2D();
        Pose2D step = new Pose2D(7.158506, 0, Math.toRadians(42));

        System.out.println("exp: " + initPose.exp(step));
        System.out.println("add: " + initPose.add(step));

    }
}
