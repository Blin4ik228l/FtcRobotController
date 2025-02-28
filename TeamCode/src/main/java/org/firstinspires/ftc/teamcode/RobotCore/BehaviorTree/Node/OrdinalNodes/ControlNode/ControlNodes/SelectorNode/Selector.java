package org.firstinspires.ftc.teamcode.RobotCore.BehaviorTree.Node.OrdinalNodes.ControlNode.ControlNodes.SelectorNode;

import org.firstinspires.ftc.teamcode.RobotCore.BehaviorTree.Node.OrdinalNodes.ControlNode.ControlNode;
import org.firstinspires.ftc.teamcode.RobotCore.BehaviorTree.States;

public class Selector extends ControlNode {
    //ControlNode (FallBack)
    //Если хоть - что вернёт success, то сам Selecetor вернёт success
    //Если все вернули failure, то сам Selecetor вернёт failure
    //Если ветвь(node) вернёт failure, то перейдёт к следующей
    @Override
    public void programm() {
        while(nodeState != States.SUCCESS){
            if(stackToDo.isEmpty()) stackNodes();//Заполняем одностороний Стэк ветвями

            tickNode();//Дёргаем ветвь для выполнения

            if(stackToDo.peek().nodeState == States.SUCCESS){
                nodeState = States.SUCCESS;
                break;//как только ветвь вернула success selector прекращает работу
            }

            if(stackToDo.peek().nodeState == States.FAILURE){
                nodeState = States.FAILURE;

                failureCount++;
                stackToDo.pop();
            }

            if(failureCount == nodes.size()) {
                nodeState = States.FAILURE;//Если все ветви вернули failure тогда selector = failure
                break;
            }
        }
    }
}
