package org.firstinspires.ftc.teamcode.Game.Robot.Logic.BehaviorTree.Node;

import org.firstinspires.ftc.teamcode.Game.Programms.Robot;
import org.firstinspires.ftc.teamcode.Game.Robot.Logic.BehaviorTree.States;

public abstract class Node {
    public Robot robot;

    public States nodeState = States.RUNNING;;

    public boolean stop = false;

    public void tickMe() {

        programm();
    }

    public void programm(){

    }

    public void stopProgramm(){

    }
}
