package org.firstinspires.ftc.teamcode.RobotCore.BehaviorTree.Pathes.TaskNodes;

import org.firstinspires.ftc.teamcode.RobotCore.BehaviorTree.Pathes.Node;
import org.firstinspires.ftc.teamcode.RobotCore.BehaviorTree.States;
import org.firstinspires.ftc.teamcode.RobotCore.TaskUtils.Tasks.OrdinaryTask;

public class TaskNode extends Node {
    public OrdinaryTask task;

    public States nodeState = task.state;

    public boolean isAdded = false;

    @Override
    public void programm() {
        if(!isAdded){
            robot.taskManager.addTaskToStack(task);
            isAdded = true;
        }

        robot.taskManager.permanentlyExecute();
    }
}
