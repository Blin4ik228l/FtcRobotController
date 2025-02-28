package org.firstinspires.ftc.teamcode.RobotCore.BehaviorTree.Node.TaskNodes.TaskSequence;

import org.firstinspires.ftc.teamcode.RobotCore.BehaviorTree.Node.Node;
import org.firstinspires.ftc.teamcode.RobotCore.BehaviorTree.States;
import org.firstinspires.ftc.teamcode.RobotCore.TaskUtils.Tasks.OrdinaryTask;

import java.util.Stack;

public abstract class TaskSequence extends Node {
    public Stack<OrdinaryTask> taskToDo = new Stack();

    public int countTasks;

    public int completCount = 0;

    @Override
    public void programm() {
        if(!taskToDo.isEmpty() && nodeState != States.FAILURE) robot.taskManager.addTaskToStack(taskToDo.peek());

        while (taskToDo.peek().state != States.SUCCESS && nodeState != States.FAILURE) {
            robot.taskManager.permanentlyExecute();

            if (taskToDo.peek().state == States.SUCCESS) {
                taskToDo.pop();//Смахиваем выполненую задачу с вершины Стэка
                completCount++;
            }
            if(completCount == countTasks) {
                nodeState = States.SUCCESS;
                break;
            }

            if(taskToDo.peek().state == States.FAILURE){
                nodeState = States.FAILURE;
                break;
            }
        }
    }
}
