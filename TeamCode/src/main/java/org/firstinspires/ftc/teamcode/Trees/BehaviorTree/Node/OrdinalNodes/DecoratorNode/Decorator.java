package org.firstinspires.ftc.teamcode.Trees.BehaviorTree.Node.OrdinalNodes.DecoratorNode;


import org.firstinspires.ftc.teamcode.Trees.BehaviorTree.Node.Node;

public abstract class Decorator extends Node {
    public Decorator(Node node){
        this.nodeToWork = node;
    }

    public Node nodeToWork;


}
