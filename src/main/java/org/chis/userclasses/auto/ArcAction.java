package org.chis.userclasses.auto;

import org.chis.sim.math.Vector2D;
import org.chis.userclasses.SwerveController;

public class ArcAction extends AbstractAction{

    Vector2D center;
    double speed;
    double angleChange;

    Double startAngle;

    public ArcAction(Vector2D center, double speed, double angleChange){
        this.center = center;
        this.speed = speed;
        this.angleChange = angleChange;
    }

    @Override
    public void runAction(SwerveController swerve) {
        if(startAngle == null){
            startAngle = swerve.gyro.getContinuousAngle();
        }

        if(Math.abs(swerve.gyro.getContinuousAngle() - startAngle) < Math.abs(angleChange)){
            swerve.nyoomAboutPoint(center, speed * Math.signum(angleChange));
        }else{
            done = true;
        }
    }

    public String toString(){
        return "center: " + center + ", speed: " + speed + ", angleChange: " + angleChange;
    }

}
