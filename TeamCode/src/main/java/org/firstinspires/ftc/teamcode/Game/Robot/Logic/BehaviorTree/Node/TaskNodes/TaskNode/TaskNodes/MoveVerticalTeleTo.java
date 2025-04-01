package org.firstinspires.ftc.teamcode.Game.Robot.Logic.BehaviorTree.Node.TaskNodes.TaskNode.TaskNodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Game.Robot.Logic.BehaviorTree.Node.TaskNodes.TaskNode.TaskNode;
import org.firstinspires.ftc.teamcode.Utils.Consts.ConstsTeleskope;
import org.firstinspires.ftc.teamcode.Game.Programms.Robot;
import org.firstinspires.ftc.teamcode.Game.Robot.Logic.TaskUtils.StandartArgs;
import org.firstinspires.ftc.teamcode.Game.Robot.Logic.TaskUtils.Tasks.OrdinaryTask;

public class MoveVerticalTeleTo extends TaskNode implements ConstsTeleskope {
    public MoveVerticalTeleTo(Robot robot, double height, LinearOpMode lin){
        super(robot, lin);
        task = new OrdinaryTask(robot.setVerticalTeleskopePos,
                new StandartArgs.verticalArgs(height, 0.95, 0.3),
                OrdinaryTask.taskStartMode.START_AFTER_PREVIOUS);
    }
}
