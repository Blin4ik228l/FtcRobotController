package org.firstinspires.ftc.teamcode.Trees.BehaviorTree.Node.OrdinalNodes.DecoratorNode.Decorators;

import org.firstinspires.ftc.teamcode.Trees.BehaviorTree.Node.Node;
import org.firstinspires.ftc.teamcode.Trees.BehaviorTree.Node.OrdinalNodes.DecoratorNode.Decorator;
import org.firstinspires.ftc.teamcode.Trees.BehaviorTree.States;


public class ForceFailure extends Decorator {
    public ForceFailure(Node node){
        super(node);
    }

    @Override
    public void programm() {
        nodeToWork.tickMe();

        if(nodeToWork.nodeState != States.RUNNING) nodeState = States.FAILURE;
    }
}
