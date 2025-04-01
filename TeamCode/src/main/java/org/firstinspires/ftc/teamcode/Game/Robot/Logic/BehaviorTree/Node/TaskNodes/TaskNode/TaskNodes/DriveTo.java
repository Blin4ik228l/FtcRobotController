package org.firstinspires.ftc.teamcode.Game.Robot.Logic.BehaviorTree.Node.TaskNodes.TaskNode.TaskNodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Game.Programms.Robot;
import org.firstinspires.ftc.teamcode.Game.Robot.Logic.BehaviorTree.Node.TaskNodes.TaskNode.TaskNode;
import org.firstinspires.ftc.teamcode.Game.Robot.Logic.TaskUtils.StandartArgs;
import org.firstinspires.ftc.teamcode.Game.Robot.Logic.TaskUtils.Tasks.OrdinaryTask;

public class DriveTo extends TaskNode {
    public DriveTo(Robot robot, StandartArgs.driveArgs driveToArgs, LinearOpMode lin){
        super(robot, lin);
        task = new OrdinaryTask(robot.driveToPosition,
                driveToArgs,
                OrdinaryTask.taskStartMode.START_AFTER_PREVIOUS);
    }
}

