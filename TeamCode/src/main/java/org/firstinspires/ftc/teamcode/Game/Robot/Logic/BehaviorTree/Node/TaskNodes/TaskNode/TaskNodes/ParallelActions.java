package org.firstinspires.ftc.teamcode.Game.Robot.Logic.BehaviorTree.Node.TaskNodes.TaskNode.TaskNodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Game.Programms.Robot;
import org.firstinspires.ftc.teamcode.Game.Robot.Logic.BehaviorTree.Node.TaskNodes.TaskNode.TaskNode;
import org.firstinspires.ftc.teamcode.Game.Robot.Logic.BehaviorTree.States;
import org.firstinspires.ftc.teamcode.Game.Robot.Logic.TaskUtils.Tasks.OrdinaryTask;

public class ParallelActions extends TaskNode {
    public ParallelActions(Robot robotT1, Robot robotT2, TaskNode parallel1, TaskNode parallel2, LinearOpMode lin) {
        super(robotT1, lin);
        this.task = parallel1.task;

        this.robot1 = robotT1;
        this.robot2 = robotT2;

        parallel1.isProgrammDisabled = true;
        parallel2.isProgrammDisabled = true;

        parallel1.task.startMode = OrdinaryTask.taskStartMode.START_WITH_PREVIOUS;
        parallel2.task.startMode = OrdinaryTask.taskStartMode.START_WITH_PREVIOUS;

        this.parallel1 = parallel1;
        this.parallel2 = parallel2;
    }
    public Robot robot2;
    public Robot robot1;
    public TaskNode parallel1;
    public TaskNode parallel2;

    @Override
    public void programm() {
        while ((task.state == States.TODO || task.state == States.RUNNING) && !lin.isStopRequested() && lin.opModeIsActive()){
            robot1.taskManager.permanentlyExecute();
        }

        nodeState = task.state;

    }
}
