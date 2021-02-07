package org.chis.sim;

import java.util.ArrayList;

import java.awt.Dimension;

import javax.swing.*;

public class Printouts extends JFrame{

    private static JPanel panel = new JPanel();

    private static ArrayList<JLabel> labels = new ArrayList<JLabel>();

    public Printouts(){
        super("Printouts");
        add(new JScrollPane(panel));
        
        panel.setLayout(new BoxLayout(panel, BoxLayout.PAGE_AXIS));
        setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);

        Dimension frameSize = new Dimension(300, 200);

        setSize(frameSize);
        setLocation((int) (GraphicSim.screenWidth - frameSize.getWidth()), 0);
        setVisible(true);
    }

    public static void clear(){
        labels.clear();
        panel.removeAll();
        panel.repaint();
    }
    
    /**
     * Prints the number and its label on the GraphicDebug window. Run this every time you want to update the number.
     * @param text What the number means.
     * @param number The number you want to print
     */
    public static void put(String text, Object object){
        for(int i = 0; i < labels.size(); i++){
            if(labels.get(i).getText().startsWith(text)){
                labels.get(i).setText(text + ": " + object);
                return;
            }
        }
        JLabel newLabel = new JLabel(text + ": " + object);
        labels.add(newLabel);
        panel.add(newLabel);
    }

    //warning if isn't here, idk why
    private static final long serialVersionUID = 1L;
}
