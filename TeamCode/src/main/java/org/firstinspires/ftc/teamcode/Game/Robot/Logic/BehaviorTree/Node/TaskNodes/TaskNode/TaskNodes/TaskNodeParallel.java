package org.firstinspires.ftc.teamcode.Game.Robot.Logic.BehaviorTree.Node.TaskNodes.TaskNode.TaskNodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Game.Programms.Robot;
import org.firstinspires.ftc.teamcode.Game.Robot.Logic.BehaviorTree.Node.TaskNodes.TaskNode.TaskNode;
import org.firstinspires.ftc.teamcode.Game.Robot.Logic.BehaviorTree.States;
import org.firstinspires.ftc.teamcode.Game.Robot.Logic.BehaviorTree.Node.Node;
import org.firstinspires.ftc.teamcode.Game.Robot.Logic.TaskUtils.Tasks.OrdinaryTask;

public class TaskNodeParallel extends Node {
    public TaskNodeParallel(Robot robot, TaskNode parallel1, TaskNode parallel2, LinearOpMode lin) {
        this.robot = robot;
        this.lin = lin;

        parallel1.task.startMode = OrdinaryTask.taskStartMode.START_WITH_PREVIOUS;
        parallel2.task.startMode = OrdinaryTask.taskStartMode.START_WITH_PREVIOUS;

        this.parallel1 = parallel1;
        this.parallel2 = parallel2;
    }

    public LinearOpMode lin;

    public TaskNode parallel1;
    public TaskNode parallel2;

    public boolean isAdded = false;

    @Override
    public void programm() {
        if(!isAdded){
            robot.taskManager.addTaskToStack(parallel1.task);
            robot.taskManager.addTaskToStack(parallel2.task);

            parallel1.isParallel = true;
            parallel2.isParallel = true;
            isAdded = true;
        }

        while ((parallel1.nodeState == States.TODO || parallel1.nodeState == States.RUNNING)
                && (parallel2.nodeState == States.TODO || parallel2.nodeState == States.RUNNING)
                && !lin.isStopRequested() && lin.opModeIsActive()){

            parallel1.tickMe();
            parallel2.tickMe();
            robot.taskManager.permanentlyExecute();
        }

        if(parallel1.nodeState == States.SUCCESS && parallel2.nodeState == States.SUCCESS){
            nodeState = States.SUCCESS;
        }

    }
}
