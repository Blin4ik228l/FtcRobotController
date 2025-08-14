package org.firstinspires.ftc.teamcode.BehaviorTree.Node.OrdinalNodes.ControlNode.ControlNodes.SequenceNode;

import org.firstinspires.ftc.teamcode.BehaviorTree.Node.OrdinalNodes.ControlNode.ControlNode;
import org.firstinspires.ftc.teamcode.BehaviorTree.States;

public class Sequence extends ControlNode {
    @Override
    public void programm() {
        if(stackToDo.isEmpty()) stackNodes();//Заполняем одностороний Стэк ветвями

        if(!stackToDo.isEmpty()) tickNode();//Дёргаем ветвь для выполнения

        if(stackToDo.peek().nodeState == States.SUCCESS){
            completCount ++;
            stackToDo.pop();
        }

        if(!stackToDo.isEmpty() && stackToDo.peek().nodeState == States.FAILURE){
            nodeState = States.FAILURE;
        }

        if(completCount == nodes.size()) {
            nodeState = States.SUCCESS;
        }
    }
}
