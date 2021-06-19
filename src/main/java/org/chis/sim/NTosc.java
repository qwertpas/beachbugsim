package org.chis.sim;

import java.io.IOException;
import java.util.List;

import com.illposed.osc.MessageSelector;
import com.illposed.osc.OSCMessageEvent;
import com.illposed.osc.OSCMessageListener;
import com.illposed.osc.messageselector.OSCPatternAddressMessageSelector;
import com.illposed.osc.transport.OSCPortIn;

public class NTosc {

    public static double x, y, z, throttle = 0;

    public static void start() {

        OSCPortIn receiver;
        try {
            receiver = new OSCPortIn(6036);
            OSCMessageListener listener = new OSCMessageListener() {
                public void acceptMessage(OSCMessageEvent event) {
                    String address = event.getMessage().getAddress();
                    List<Object> data = event.getMessage().getArguments();

                    if(address.equals("/syntien/joystick/1/2dslider1")){
                        x = Double.valueOf(data.get(0).toString()) * 2 - 1;
                        y = -(Double.valueOf(data.get(1).toString()) * 2 - 1);
                    }
                    if(address.equals("/syntien/joystick/1/slider1")){
                        throttle = Double.valueOf(data.get(0).toString()) * 2 - 1;
                    }
                    if(address.equals("/syntien/joystick/1/slider2")){
                        z = Double.valueOf(data.get(0).toString()) * 2 - 1;
                    }

                }
            };
            MessageSelector[] selectors = {
                new OSCPatternAddressMessageSelector("/syntien/joystick/1/2dslider1"),
                new OSCPatternAddressMessageSelector("/syntien/joystick/1/slider1"),
                new OSCPatternAddressMessageSelector("/syntien/joystick/1/slider2"),
            };

            for(MessageSelector selector : selectors){
                receiver.getDispatcher().addListener(selector, listener);
            }
    
            receiver.startListening();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }


    public static void main(String[] args) {
        NTosc.start();

        while(true){
            System.out.println("waiting");

            try {
                Thread.sleep(100);
                System.out.println(x + ", " + y + ", " + z + ", " + throttle);

            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }
}
