package org.firstinspires.ftc.teamcode.RobotCore.BehaviorTree.Pathes.Nodes.Main;

import org.firstinspires.ftc.teamcode.OpModes.Robot;
import org.firstinspires.ftc.teamcode.RobotCore.BehaviorTree.Pathes.Node;
import org.firstinspires.ftc.teamcode.RobotCore.BehaviorTree.Pathes.Nodes.Sequence;

public class Root extends Node {
    Sequence root;

    public Root(Robot robot){
        this.robot = robot;

        root = new Sequence();
    }

    @Override
    public void tickMe() {
        root.tickMe();
    }

    public void add(Node children){
        root.addNode(children);
    }
}