package org.firstinspires.ftc.teamcode.RobotCore.BehaviorTree.Node.OrdinalNodes.ControlNode.ControlNodes.SequenceNode;

import org.firstinspires.ftc.teamcode.RobotCore.BehaviorTree.Node.OrdinalNodes.ControlNode.ControlNode;
import org.firstinspires.ftc.teamcode.RobotCore.BehaviorTree.States;

public class Sequence extends ControlNode {
    @Override
    public void programm() {
        if(stackToDo.isEmpty()) stackNodes();//Заполняем одностороний Стэк ветвями

        tickNode();//Дёргаем ветвь для выполнения

        if(stackToDo.peek().nodeState == States.SUCCESS){
            completCount ++;
            stackToDo.pop();
        }

        if(stackToDo.peek().nodeState == States.FAILURE){
            nodeState = States.FAILURE;
        }

        if(completCount == nodes.size()) {
            nodeState = States.SUCCESS;
        }
    }
}
