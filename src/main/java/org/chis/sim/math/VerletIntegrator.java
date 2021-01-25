package org.chis.sim.math;

// Time-Corrected Verlet Integrator
public class VerletIntegrator {

    public double pos;
    double pos_prev;

    public double vel;

    double dt_prev;

    public VerletIntegrator(double pos_init, double vel_init, double acc_init, double dt_init){
        pos_prev = pos_init;
        dt_prev = dt_init;
        pos = pos_prev + vel_init * dt_prev + 0.5 * acc_init * dt_prev * dt_prev;
        vel = vel_init + acc_init * dt_prev;
    }

    public void update(double acc, double dt){
        // double pos_next = pos + (pos - pos_prev)*(dt / dt_prev) + acc * dt * dt;
        // vel = (pos_next - pos_prev) / (dt + dt_prev);
        
        // pos_prev = pos;
        // pos = pos_next;
        // dt_prev = dt;

        vel += acc * dt;
        pos += vel * dt;
    }
}
