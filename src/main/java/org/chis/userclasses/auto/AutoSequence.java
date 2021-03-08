package org.chis.userclasses.auto;

import java.util.ArrayList;

public class AutoSequence {

    ArrayList<AbstractAction> actions;
    int actionIndex;
    AbstractAction action;
    
    public AutoSequence(AbstractAction... actions_arr){

        //convert array into arraylist
        actions = new ArrayList<AbstractAction>();
        for(AbstractAction action_new : actions_arr){
            actions.add(action_new);
        }

        actionIndex = 0;
    }

    public void addAction(AbstractAction action_new){
        actions.add(action_new);
    }

    public void run(){
        action = actions.get(actionIndex);
        if(!action.done){
            action.run();
        }else{
            actionIndex++;
        }
    }

}
