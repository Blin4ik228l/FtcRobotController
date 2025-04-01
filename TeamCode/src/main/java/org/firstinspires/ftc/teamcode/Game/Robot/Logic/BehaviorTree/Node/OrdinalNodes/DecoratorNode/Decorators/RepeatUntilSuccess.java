package org.firstinspires.ftc.teamcode.Game.Robot.Logic.BehaviorTree.Node.OrdinalNodes.DecoratorNode.Decorators;

import org.firstinspires.ftc.teamcode.Game.Robot.Logic.BehaviorTree.States;
import org.firstinspires.ftc.teamcode.Game.Robot.Logic.BehaviorTree.Node.Node;
import org.firstinspires.ftc.teamcode.Game.Robot.Logic.BehaviorTree.Node.OrdinalNodes.DecoratorNode.Decorator;

public class RepeatUntilSuccess extends Decorator {
    public RepeatUntilSuccess(Node node, boolean opModeIsActive, boolean isStopRequest) {
        super(node);
        this.opModeIsActive = opModeIsActive;
        this.isStopRequest = isStopRequest;
    }
    boolean opModeIsActive;
    boolean isStopRequest;

    @Override
    public void programm() {
        while(nodeToWork.nodeState != States.SUCCESS && opModeIsActive && !isStopRequest){
            nodeToWork.tickMe();
        }
        nodeState = nodeToWork.nodeState;
    }
}
