package org.firstinspires.ftc.teamcode.RobotCore.BehaviorTree.Pathes.TaskNodes.ActionNodes;

import org.firstinspires.ftc.teamcode.Consts.ConstsTeleskope;
import org.firstinspires.ftc.teamcode.RobotCore.BehaviorTree.Pathes.TaskNodes.TaskNode;
import org.firstinspires.ftc.teamcode.RobotCore.TaskUtils.StandartArgs;
import org.firstinspires.ftc.teamcode.RobotCore.TaskUtils.Tasks.OrdinaryTask;

public class DownVerticalTeleTo extends TaskNode implements ConstsTeleskope {
    public DownVerticalTeleTo(double height){
        this.task = new OrdinaryTask(robot.setTeleskopePos,
                new StandartArgs.teleskopeArgs(height, CLOSE_POS_HORIZONTAL, 0.7),
                OrdinaryTask.taskStartMode.START_AFTER_PREVIOUS);
    }

    @Override
    public void tickMe() {
        programm();
    }
}
