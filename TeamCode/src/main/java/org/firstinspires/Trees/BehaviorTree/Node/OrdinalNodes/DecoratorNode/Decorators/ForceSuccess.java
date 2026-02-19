package org.firstinspires.Trees.BehaviorTree.Node.OrdinalNodes.DecoratorNode.Decorators;

import org.firstinspires.Trees.BehaviorTree.Node.Node;
import org.firstinspires.Trees.BehaviorTree.Node.OrdinalNodes.DecoratorNode.Decorator;
import org.firstinspires.Trees.BehaviorTree.States;


public class ForceSuccess extends Decorator {
    public ForceSuccess(Node node) {
        super(node);
    }

    @Override
    public void programm() {
        nodeToWork.tickMe();

        if(nodeToWork.nodeState != States.RUNNING) nodeState = States.SUCCESS;
    }
}
