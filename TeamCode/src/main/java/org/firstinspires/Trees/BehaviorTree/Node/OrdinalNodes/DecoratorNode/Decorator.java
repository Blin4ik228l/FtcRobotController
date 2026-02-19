package org.firstinspires.Trees.BehaviorTree.Node.OrdinalNodes.DecoratorNode;


import org.firstinspires.Trees.BehaviorTree.Node.Node;

public abstract class Decorator extends Node {
    public Decorator(Node node){
        this.nodeToWork = node;
    }

    public Node nodeToWork;


}
