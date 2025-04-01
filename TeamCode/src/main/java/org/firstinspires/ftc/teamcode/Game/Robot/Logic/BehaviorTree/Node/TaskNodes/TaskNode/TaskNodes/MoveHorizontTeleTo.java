package org.firstinspires.ftc.teamcode.Game.Robot.Logic.BehaviorTree.Node.TaskNodes.TaskNode.TaskNodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Game.Robot.Logic.BehaviorTree.Node.TaskNodes.TaskNode.TaskNode;
import org.firstinspires.ftc.teamcode.Utils.Consts.ConstsTeleskope;
import org.firstinspires.ftc.teamcode.Game.Programms.Robot;
import org.firstinspires.ftc.teamcode.Game.Robot.Logic.TaskUtils.StandartArgs;
import org.firstinspires.ftc.teamcode.Game.Robot.Logic.TaskUtils.Tasks.OrdinaryTask;

public class MoveHorizontTeleTo extends TaskNode implements ConstsTeleskope {
    public MoveHorizontTeleTo(Robot robot, double horizontalPos, LinearOpMode lin){
        super(robot, lin);
        task = new OrdinaryTask(robot.setHorizontalTeleskopePos,
                new StandartArgs.horizontalArgs(horizontalPos, 0.5),
                OrdinaryTask.taskStartMode.START_AFTER_PREVIOUS);
    }
}
