package org.firstinspires.ftc.teamcode.RobotCore.BehaviorTree.Node.OrdinalNodes.DecoratorNode.Decorators;

import org.firstinspires.ftc.teamcode.RobotCore.BehaviorTree.Node.OrdinalNodes.DecoratorNode.Decorator;
import org.firstinspires.ftc.teamcode.RobotCore.BehaviorTree.Node.Node;
import org.firstinspires.ftc.teamcode.RobotCore.BehaviorTree.States;

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
