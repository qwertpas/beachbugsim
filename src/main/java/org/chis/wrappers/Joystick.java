package org.chis.wrappers;

import org.chis.sim.Controls;
import org.chis.sim.NTosc;

public class Joystick {
    int port = 0;

    public Joystick(int port){
        if(port > 1 || port < 0){
            System.out.println("Use port 0 for USB joystick or cursor position, port 1 for phone app");
            System.exit(0);
        }
        if(port == 1){
            NTosc.start();
            System.out.println("phone");
        }
        this.port = port;
    }

    public double getX(){
        if(port == 0){
            return Controls.rawX;
        }else{
            return NTosc.x;
        }
    }

    public double getY(){
        if(port == 0){
            return Controls.rawY;
        }else{
            return NTosc.y;
        }
    }

    public double getZ(){
        if(port == 0){
            return Controls.rawZ;
        }else{
            return NTosc.z;
        }
    }

    public double getThrottle(){
        if(port == 0){
            return Controls.slider;
        }else{
            return NTosc.throttle;
        }
    }
}
