package org.firstinspires.ftc.teamcode.RobotCore.BehaviorTree.Node.TaskNodes.TaskNode.TaskNodes;

import org.firstinspires.ftc.teamcode.RobotCore.BehaviorTree.Node.TaskNodes.TaskNode.TaskNode;
import org.firstinspires.ftc.teamcode.RobotCore.TaskUtils.StandartArgs;
import org.firstinspires.ftc.teamcode.RobotCore.TaskUtils.Tasks.OrdinaryTask;

public class DriveTo extends TaskNode {
    public DriveTo(StandartArgs.driveArgs driveToArgs){
        task = new OrdinaryTask(robot.driveToPosition,
                driveToArgs,
                OrdinaryTask.taskStartMode.START_AFTER_PREVIOUS);

    }
}
