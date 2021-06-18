package org.chis.wrappers;

import org.chis.sim.Controls;

public class Joystick {
    public Joystick(int port){

    }

    public double getX(){
        return Controls.rawX;
    }

    public double getY(){
        return Controls.rawY;
    }

    public double getZ(){
        return Controls.rawZ;
    }

    public double getThrottle(){
        return Controls.slider;
    }
}
