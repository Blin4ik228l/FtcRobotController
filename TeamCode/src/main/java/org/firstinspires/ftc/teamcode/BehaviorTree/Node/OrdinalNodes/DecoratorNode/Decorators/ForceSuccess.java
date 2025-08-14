package org.firstinspires.ftc.teamcode.BehaviorTree.Node.OrdinalNodes.DecoratorNode.Decorators;

import org.firstinspires.ftc.teamcode.BehaviorTree.Node.Node;
import org.firstinspires.ftc.teamcode.BehaviorTree.Node.OrdinalNodes.DecoratorNode.Decorator;
import org.firstinspires.ftc.teamcode.BehaviorTree.States;


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
