package org.firstinspires.ftc.teamcode.RobotCore.BehaviorTree.Node.TaskNodes.TaskNode.TaskNodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.OpModes.Robot;
import org.firstinspires.ftc.teamcode.RobotCore.BehaviorTree.Node.TaskNodes.TaskNode.TaskNode;
import org.firstinspires.ftc.teamcode.RobotCore.TaskUtils.StandartArgs;
import org.firstinspires.ftc.teamcode.RobotCore.TaskUtils.Tasks.OrdinaryTask;

public class DriveTo extends TaskNode {
    public DriveTo(Robot robot, StandartArgs.driveArgs driveToArgs, LinearOpMode lin){
        super(robot, lin);
        task = new OrdinaryTask(robot.driveToPosition,
                driveToArgs,
                OrdinaryTask.taskStartMode.START_AFTER_PREVIOUS);
    }
}

