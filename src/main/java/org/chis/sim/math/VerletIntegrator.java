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

        // if accel makes vel go past zero, then just make vel 0 to avoid friction vibrations
        if(Math.signum(vel + acc * dt) != Math.signum(vel) && vel != 0){
            vel = 0;
        }else{
            vel += acc * dt;
        }
        pos += vel * dt;
    }

    public static void main(String[] args) {
        double dt = 0.1;

        VerletIntegrator x = new VerletIntegrator(0, 0, 0, dt);

        x.update(1, dt);
        System.out.println(x.vel);

        x.update(1, dt);
        System.out.println(x.vel);

        x.update(-1, dt);
        System.out.println(x.vel);

        x.update(-1, dt);
        System.out.println(x.vel);

        x.update(-1, dt);
        System.out.println(x.vel);
    }
}
