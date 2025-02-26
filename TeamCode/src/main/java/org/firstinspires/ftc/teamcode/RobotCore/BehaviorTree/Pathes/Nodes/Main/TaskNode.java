package org.firstinspires.ftc.teamcode.RobotCore.BehaviorTree.Pathes.Nodes.Main;

import org.firstinspires.ftc.teamcode.RobotCore.BehaviorTree.Pathes.Nodes.Node;
import org.firstinspires.ftc.teamcode.RobotCore.BehaviorTree.Pathes.States;
import org.firstinspires.ftc.teamcode.RobotCore.TaskUtils.TaskManager;
import org.firstinspires.ftc.teamcode.RobotCore.TaskUtils.Tasks.OrdinaryTask;

public class TaskNode extends Node {
    public OrdinaryTask task;
    public TaskManager taskManager;

    public States childrenState = States.WAITING;

    public TaskNode(TaskManager taskManager, OrdinaryTask task){
        this.task = task;
        this.taskManager = taskManager;

        taskManager.addTask(task);
    }

    @Override
    public void tickMe(){
        if(childrenState != States.RUNNING && task.state != States.FAILURE) childrenState = States.RUNNING;

        taskManager.forAuto();

        childrenState = task.state;
    }
}
