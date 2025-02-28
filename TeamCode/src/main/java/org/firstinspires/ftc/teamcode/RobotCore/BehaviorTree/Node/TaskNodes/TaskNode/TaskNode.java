package org.firstinspires.ftc.teamcode.RobotCore.BehaviorTree.Node.TaskNodes.TaskNode;

import org.firstinspires.ftc.teamcode.RobotCore.BehaviorTree.Node.Node;
import org.firstinspires.ftc.teamcode.RobotCore.TaskUtils.Tasks.OrdinaryTask;

public abstract class TaskNode extends Node {
    public OrdinaryTask task;

    public boolean isAdded = false;

    @Override
    public void programm() {
        if(!isAdded){
            robot.taskManager.addTaskToStack(task);
            isAdded = true;
        }

        robot.taskManager.permanentlyExecute();

        nodeState = task.state;
    }
}
