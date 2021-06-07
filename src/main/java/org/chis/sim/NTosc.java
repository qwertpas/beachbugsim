package org.chis.sim;

import java.io.IOException;
import java.util.List;

import org.chis.sim.math.Pose2D;
import org.chis.sim.math.Vector2D;
import org.chis.sim.math.Vector2D.Type;

import com.illposed.osc.MessageSelector;
import com.illposed.osc.OSCMessageEvent;
import com.illposed.osc.OSCMessageListener;
import com.illposed.osc.messageselector.OSCPatternAddressMessageSelector;
import com.illposed.osc.transport.OSCPortIn;

public class NTosc {

    public static Vector2D finger1 = new Vector2D(0, 0, Type.CARTESIAN);
    public static Vector2D finger2 = new Vector2D(1, 1, Type.CARTESIAN);

    public static void start() {

        OSCPortIn receiver;
        try {
            receiver = new OSCPortIn(6036);
            OSCMessageListener listener = new OSCMessageListener() {
                public void acceptMessage(OSCMessageEvent event) {
                    List<Object> data = event.getMessage().getArguments();
                    double x = Double.valueOf(data.get(0).toString());
                    double y = Double.valueOf(data.get(1).toString());

                    Vector2D temp = new Vector2D(x, y, Type.CARTESIAN);

                    if(temp.dist(finger1) < temp.dist(finger2)){
                        finger1 = temp;
                    }else{
                        finger2 = temp;
                    }


                }
            };
            MessageSelector[] selectors = {
                new OSCPatternAddressMessageSelector("/syntien/untitled/1/2dslider1"),
                new OSCPatternAddressMessageSelector("/syntien/untitled/1/spinner1"),
                new OSCPatternAddressMessageSelector("/syntien/basic/1/touchpad1/press"),
            };

            for(MessageSelector selector : selectors){
                receiver.getDispatcher().addListener(selector, listener);
            }
    
            receiver.startListening();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public static Vector2D get1(){
        return finger1.scalarMult(2).subtract(new Vector2D(1, 1, Type.CARTESIAN));
    }

    public static Vector2D get2(){
        return finger2.scalarMult(2).subtract(new Vector2D(1, 1, Type.CARTESIAN));
    }

    public static Pose2D getPose(){
        return new Pose2D(get1().add(get2()).scalarDiv(2), get1().subtract(get2()).rotate90().getAngle());
    }

    public static void main(String[] args) {
        NTosc.start();

        while(true){
            System.out.println("waiting");

            try {
                Thread.sleep(100);
                System.out.println(get1() + " : " + get2());

            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }
}
