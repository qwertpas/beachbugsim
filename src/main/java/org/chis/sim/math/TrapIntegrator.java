package org.chis.sim.math;

public class TrapIntegrator {

    public double preVel = 0;
    public double newVel = 0;

    public double pos;

    public TrapIntegrator(double initPos){
        pos = initPos;
    }

    public void update(double vel, double dt){
        newVel = vel;

        pos = pos + 0.5 * (newVel + preVel) * dt;

        preVel = vel;
    }
}
