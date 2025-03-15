package org.firstinspires.ftc.teamcode.RobotCore.BehaviorTree.Node.TaskNodes.TaskNode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.apache.commons.math3.geometry.euclidean.twod.Line;
import org.firstinspires.ftc.teamcode.OpModes.Robot;
import org.firstinspires.ftc.teamcode.RobotCore.BehaviorTree.Node.Node;
import org.firstinspires.ftc.teamcode.RobotCore.BehaviorTree.States;
import org.firstinspires.ftc.teamcode.RobotCore.TaskUtils.Tasks.OrdinaryTask;

public abstract class TaskNode extends Node {
    public TaskNode(Robot robot, LinearOpMode lin) {
        this.robot = robot;
        this.lin = lin;
    }

    public LinearOpMode lin;
    public OrdinaryTask task;

    public boolean isAdded = false;

    public boolean isProgrammDisabled = false;

    @Override
    public void programm() {
        if(!isProgrammDisabled) {
            if (!isAdded) {
                robot.taskManager.addTaskToStack(task);
//                robot.robotStatusHandler.tasksToDo.add(task);
                isAdded = true;
            }

            while ((task.state == States.TODO || task.state == States.RUNNING) && !lin.isStopRequested() && lin.opModeIsActive()){
                robot.taskManager.permanentlyExecute();
            }

            nodeState = task.state;
        }
    }
}
