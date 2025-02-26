package org.firstinspires.ftc.teamcode.RobotCore.BehaviorTree.Pathes.Nodes.Main;

import org.firstinspires.ftc.teamcode.RobotCore.BehaviorTree.Pathes.States;

public class Root {
    Sequence root = new Sequence();

    public void addControlNode(){

    }

    public void add(TaskNode children){
        root.addChildren(children);
    }

    public void tickSequence(){
        while(root.sequenceState != States.SUCCESS){
            root.tickMe();

            if(root.sequenceState == States.FAILURE) break;
        }
    }
}