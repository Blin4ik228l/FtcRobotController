package org.firstinspires.ftc.teamcode.Trees.BehaviorTree;

import org.firstinspires.ftc.teamcode.Trees.BehaviorTree.Node.Node;
import org.firstinspires.ftc.teamcode.TaskAndArgs.Task;

public class TaskNode extends Node {
    public TaskNode(Task task){
        this.task = task;
    }

    @Override
    public void programm() {
//        if(!task.handler.isDone){
//            task.startDoing();
//        }else{
//            nodeState = States.SUCCESS;
//        }
    }
}
