package org.firstinspires.ftc.teamcode.RobotCore.BehaviorTree.Pathes.TaskNodes.ActionNodes;

import org.firstinspires.ftc.teamcode.RobotCore.BehaviorTree.Pathes.TaskNodes.TaskNode;
import org.firstinspires.ftc.teamcode.RobotCore.TaskUtils.StandartArgs;
import org.firstinspires.ftc.teamcode.RobotCore.TaskUtils.Tasks.OrdinaryTask;

public class DriveTo extends TaskNode {
    public DriveTo(StandartArgs.driveArgs driveToArgs){
        this.task = new OrdinaryTask(robot.driveToPosition,
                driveToArgs,
                OrdinaryTask.taskStartMode.START_AFTER_PREVIOUS);
    }

    @Override
    public void tickMe() {
        programm();
    }
}
