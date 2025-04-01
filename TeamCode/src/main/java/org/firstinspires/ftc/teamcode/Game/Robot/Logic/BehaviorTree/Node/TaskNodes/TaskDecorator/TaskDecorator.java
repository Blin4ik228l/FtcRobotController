package org.firstinspires.ftc.teamcode.Game.Robot.Logic.BehaviorTree.Node.TaskNodes.TaskDecorator;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Game.Programms.Robot;
import org.firstinspires.ftc.teamcode.Game.Robot.Logic.BehaviorTree.Node.TaskNodes.TaskNode.TaskNode;

public class TaskDecorator extends TaskNode {
    public TaskDecorator(Robot robot, LinearOpMode lin) {
        super(robot, lin);
    }

    public TaskNode nodeToWork;
}
