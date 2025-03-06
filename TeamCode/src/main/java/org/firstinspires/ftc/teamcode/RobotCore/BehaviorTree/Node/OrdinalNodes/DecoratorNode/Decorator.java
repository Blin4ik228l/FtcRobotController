package org.firstinspires.ftc.teamcode.RobotCore.BehaviorTree.Node.OrdinalNodes.DecoratorNode;

import org.firstinspires.ftc.teamcode.RobotCore.BehaviorTree.Node.Node;

public abstract class Decorator extends Node {
    public Decorator(Node node){
        this.nodeToWork = node;
    }

    public Node nodeToWork;


}
