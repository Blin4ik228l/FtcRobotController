package org.firstinspires.ftc.teamcode.BehaviorTree.Node.OrdinalNodes.DecoratorNode.Decorators;

import org.firstinspires.ftc.teamcode.BehaviorTree.Node.Node;
import org.firstinspires.ftc.teamcode.BehaviorTree.Node.OrdinalNodes.DecoratorNode.Decorator;
import org.firstinspires.ftc.teamcode.BehaviorTree.States;


public class Repeat extends Decorator {
    public Repeat(Node node, int times) {
        super(node);
    }
    public int times;
    public int countTimes;

    @Override
    public void programm() {
        while (times != countTimes){
            nodeToWork.tickMe();

            if(nodeToWork.nodeState == States.SUCCESS){
                countTimes++;
            }
            if(countTimes == times){
                nodeState = States.SUCCESS;
                break;
            }
            if(nodeToWork.nodeState == States.FAILURE){
                nodeState = States.FAILURE;
                break;
            }
        }
    }
}
