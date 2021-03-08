package org.chis.userclasses.auto;

import org.chis.sim.math.Vector2D;
import org.chis.userclasses.SwerveController;

public class LineAction extends AbstractAction{

    Vector2D endpoint;
    double speed;
    double toleranceRadius;

    public LineAction(Vector2D endpoint, double speed, double toleranceRadius){
        this.endpoint = endpoint;
        this.speed = speed;
        this.toleranceRadius = toleranceRadius;
    }

    @Override
    public void runAction(SwerveController swerve) {
        if(swerve.odo.robotPose.dist(endpoint) > toleranceRadius){
            swerve.nyoomToPoint(endpoint, speed);
        }else{
            done = true;
        }
    }


    public String toString(){
        return "endpoint: " + endpoint + ", speed: " + speed + ", toleranceRadius: " + toleranceRadius;
    }

}
