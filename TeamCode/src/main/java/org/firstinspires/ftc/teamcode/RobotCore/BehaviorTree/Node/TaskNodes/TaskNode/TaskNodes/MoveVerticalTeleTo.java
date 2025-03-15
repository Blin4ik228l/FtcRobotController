package org.firstinspires.ftc.teamcode.RobotCore.BehaviorTree.Node.TaskNodes.TaskNode.TaskNodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Consts.ConstsTeleskope;
import org.firstinspires.ftc.teamcode.OpModes.Robot;
import org.firstinspires.ftc.teamcode.RobotCore.BehaviorTree.Node.TaskNodes.TaskNode.TaskNode;
import org.firstinspires.ftc.teamcode.RobotCore.TaskUtils.StandartArgs;
import org.firstinspires.ftc.teamcode.RobotCore.TaskUtils.Tasks.OrdinaryTask;

public class MoveVerticalTeleTo extends TaskNode implements ConstsTeleskope {
    public MoveVerticalTeleTo(Robot robot, double height, LinearOpMode lin){
        super(robot, lin);
        task = new OrdinaryTask(robot.setTeleskopePos,
                new StandartArgs.teleskopeArgs(height, CLOSE_POS_HORIZONTAL, 0.7, 1),
                OrdinaryTask.taskStartMode.START_AFTER_PREVIOUS);
    }
}
