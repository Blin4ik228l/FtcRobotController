package org.firstinspires.ftc.teamcode.RobotCore.BehaviorTree.Node.TaskNodes.TaskNode.TaskNodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Consts.ConstsTeleskope;
import org.firstinspires.ftc.teamcode.OpModes.Robot;
import org.firstinspires.ftc.teamcode.RobotCore.BehaviorTree.Node.TaskNodes.TaskNode.TaskNode;
import org.firstinspires.ftc.teamcode.RobotCore.TaskUtils.StandartArgs;
import org.firstinspires.ftc.teamcode.RobotCore.TaskUtils.Tasks.OrdinaryTask;

public class MoveHorizontTeleTo extends TaskNode implements ConstsTeleskope {
    public MoveHorizontTeleTo(Robot robot, double horizontalPos, LinearOpMode lin){
        super(robot, lin);
        task = new OrdinaryTask(robot.setHorizontalTeleskopePos,
                new StandartArgs.horizontalArgs(horizontalPos, 0.3),
                OrdinaryTask.taskStartMode.START_AFTER_PREVIOUS);
    }
}
