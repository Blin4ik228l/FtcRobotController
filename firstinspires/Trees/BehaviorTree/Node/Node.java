package org.firstinspires.Trees.BehaviorTree.Node;

import org.firstinspires.Trees.BehaviorTree.States;
import org.firstinspires.TaskAndArgs.Task;

public abstract class Node {
    public States nodeState = States.RUNNING;;
    public Task task;
    public boolean stop = false;

    public void tickMe() {
        if(stop){
            return;
        }
        programm();
    }

    public void programm(){

    }

    public void stopProgramm(){

    }
}
