package org.firstinspires.ftc.teamcode.RobotCore.BehaviorTree.Node.TaskNodes.TaskNode.TaskNodes;

import org.firstinspires.ftc.teamcode.OpModes.Robot;
import org.firstinspires.ftc.teamcode.RobotCore.BehaviorTree.Node.TaskNodes.TaskNode.TaskNode;
import org.firstinspires.ftc.teamcode.RobotCore.TaskUtils.StandartArgs;
import org.firstinspires.ftc.teamcode.RobotCore.TaskUtils.Tasks.OrdinaryTask;

public class Wait extends TaskNode {
    public Wait(Robot robot, StandartArgs.robotSleep robotSleep) {
        super(robot);
        task = new OrdinaryTask(robot.robotSleep,
                robotSleep,
                OrdinaryTask.taskStartMode.START_AFTER_PREVIOUS);
    }


}
