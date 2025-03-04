package org.firstinspires.ftc.teamcode.RobotCore.BehaviorTree.Node.OrdinalNodes.DecoratorNode.Decorators;

import org.firstinspires.ftc.teamcode.RobotCore.BehaviorTree.Node.Node;
import org.firstinspires.ftc.teamcode.RobotCore.BehaviorTree.Node.OrdinalNodes.DecoratorNode.Decorator;
import org.firstinspires.ftc.teamcode.RobotCore.BehaviorTree.States;

public class RepeatUntilSuccess extends Decorator {
    public RepeatUntilSuccess(Node node) {
        super(node);
    }

    @Override
    public void programm() {
        while(nodeToWork.nodeState != States.SUCCESS){
            nodeToWork.tickMe();
        }
    }
}
