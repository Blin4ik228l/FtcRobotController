package org.firstinspires.ftc.teamcode.RobotCore.BehaviorTree;

import org.firstinspires.ftc.teamcode.RobotCore.BehaviorTree.Node.Node;
import org.firstinspires.ftc.teamcode.RobotCore.BehaviorTree.Node.OrdinalNodes.ControlNode.ControlNodes.SequenceNode.Sequence;

public class Root {
    public Sequence root;

    public Root(){
        root = new Sequence();
    }

    public void tick() {
        root.tickMe();
    }

    public void add(Node children){
        root.addNode(children);
    }
}