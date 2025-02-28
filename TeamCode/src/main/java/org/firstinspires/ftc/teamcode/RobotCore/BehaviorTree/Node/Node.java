package org.firstinspires.ftc.teamcode.RobotCore.BehaviorTree.Node;

import org.firstinspires.ftc.teamcode.OpModes.Robot;
import org.firstinspires.ftc.teamcode.RobotCore.BehaviorTree.States;

public abstract class Node {
    public Robot robot;

    public States nodeState;

    public void tickMe() {
        if(nodeState == null) nodeState = States.RUNNING;

        programm();
    }

    public void programm(){

    }
}
