package org.firstinspires.ftc.teamcode.RobotCore.BehaviorTree.Pathes.Nodes;

import org.firstinspires.ftc.teamcode.RobotCore.BehaviorTree.Pathes.ControlNode;
import org.firstinspires.ftc.teamcode.RobotCore.BehaviorTree.States;

public class Sequence extends ControlNode {

    @Override
    public void tickMe() {
        programm();
    }

    @Override
    public void programm() {
        if(nodeState != States.RUNNING && nodeState != States.FAILURE) nodeState = States.RUNNING;

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
