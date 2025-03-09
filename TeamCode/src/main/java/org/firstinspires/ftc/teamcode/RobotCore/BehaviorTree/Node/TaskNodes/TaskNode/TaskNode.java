package org.firstinspires.ftc.teamcode.RobotCore.BehaviorTree.Node.TaskNodes.TaskNode;

import org.firstinspires.ftc.teamcode.OpModes.Robot;
import org.firstinspires.ftc.teamcode.RobotCore.BehaviorTree.Node.Node;
import org.firstinspires.ftc.teamcode.RobotCore.TaskUtils.Tasks.OrdinaryTask;

public abstract class TaskNode extends Node {
    public TaskNode(Robot robot) {
        this.robot = robot;
    }

    public OrdinaryTask task;

    public boolean isAdded = false;

    public boolean isProgrammDisabled = false;

    @Override
    public void programm() {
        if(!isProgrammDisabled) {
            if (!isAdded) {
                robot.taskManager.addTaskToStack(task);
                robot.robotStatusHandler.tasksToDo.add(task);
                isAdded = true;
            }

            robot.taskManager.permanentlyExecute();

            nodeState = task.state;
        }
    }
}
