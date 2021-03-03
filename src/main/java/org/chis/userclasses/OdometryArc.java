package org.chis.userclasses;

import org.chis.sim.math.Pose2D;
import org.chis.sim.math.Vector2D;
import org.chis.sim.math.Vector2D.Type;
import org.ejml.simple.*;

public class OdometryArc {
    
    Pose2D robotPose;
    Pose2D[] placements;

    public OdometryArc(Pose2D robotPoseInit, Pose2D ... placements){
        this.robotPose = robotPoseInit;
        this.placements = placements;
    }

    public void update(WheelData ... wheelData){
        if(wheelData.length != placements.length){
            throw new RuntimeException("data not same length");
        }

        SimpleMatrix A = new SimpleMatrix(placements.length, 3);
        SimpleMatrix y = new SimpleMatrix(placements.length, 1);

        for(int wheelIndex = 0; wheelIndex < placements.length; wheelIndex++){
            double dist = wheelData[wheelIndex].dist;
            double angle = wheelData[wheelIndex].angle + placements[wheelIndex].ang;

            A.setRow(wheelIndex, 0, Math.sin(angle), -Math.cos(angle), -dist);
            y.setRow(wheelIndex, 0, placements[wheelIndex].x * Math.sin(angle) - placements[wheelIndex].y * Math.cos(angle));
        }

        System.out.println("A: " + A);
        System.out.println("y: " + y);


        SimpleMatrix AT = A.transpose();
        SimpleMatrix x_t = AT.mult(A).solve(AT.mult(y));
        SimpleMatrix x = A.solve(y);

        System.out.println("x: " + x);
        System.out.println("x_t: " + x_t);

        Vector2D toTurnCenter = new Vector2D(x.get(0), x.get(1), Type.CARTESIAN);
        double angleMovement = 1 / x.get(2);

        System.out.println("toTurnCenter: " + toTurnCenter);
        System.out.println("angleMovement: " + angleMovement);


        Vector2D robotStep = toTurnCenter.subtract(toTurnCenter.rotate(-angleMovement));
        robotPose = robotPose.add(robotStep).rotateAng(-angleMovement);

        System.out.println("robotStep: " + robotStep);
        System.out.println("robotpose: " + robotPose);
    }

    public static class WheelData{
        double angle;
        double dist;
        public WheelData(double angle, double dist){
            this.angle = angle;
            this.dist = dist;
        }
    }

    public static void main(String[] args) {
        OdometryArc odo = new OdometryArc(
            new Pose2D(+0, +0, +0),

            new Pose2D(+1, +1, +0),
            new Pose2D(-1, +1, +0),
            new Pose2D(-1, -1, +0),
            new Pose2D(+1, -1, +0)
        );

        double toRad = Math.PI / 180;

        // odo.update(
        //     new WheelData(7.8 * toRad, 2.724),
        //     new WheelData(34.4 * toRad, 3.272),
        //     new WheelData(56.6 * toRad, 2.216),
        //     new WheelData(16.9 * toRad, 1.276)
        // );

        odo.update(
            new WheelData(0, 2),
            new WheelData(0, 1),
            new WheelData(0.1, 2),
            new WheelData(0.002, 2)
        );

    }
}

