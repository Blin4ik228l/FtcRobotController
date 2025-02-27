package org.firstinspires.ftc.teamcode.RobotCore.BehaviorTree.Pathes.TaskNodes.ActionNodes;

import org.firstinspires.ftc.teamcode.Consts.ConstsTeleskope;
import org.firstinspires.ftc.teamcode.RobotCore.BehaviorTree.Pathes.TaskNodes.TaskNode;
import org.firstinspires.ftc.teamcode.RobotCore.TaskUtils.StandartArgs;
import org.firstinspires.ftc.teamcode.RobotCore.TaskUtils.Tasks.OrdinaryTask;

public class MoveHorizontTeleTo extends TaskNode implements ConstsTeleskope {
    public MoveHorizontTeleTo(double horizontalPos){
        this.task = new OrdinaryTask(robot.setTeleskopePos,
                new StandartArgs.teleskopeArgs(robot.teleSkope.getHeight(),horizontalPos, 0),
                OrdinaryTask.taskStartMode.START_AFTER_PREVIOUS);
    }

    @Override
    public void tickMe() {
        programm();
    }
}
