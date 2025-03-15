package org.firstinspires.ftc.teamcode.RobotCore.BehaviorTree.Node.TaskNodes.TaskSequence.TaskSequences;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Consts.ConstsTeleskope;
import org.firstinspires.ftc.teamcode.OpModes.Robot;
import org.firstinspires.ftc.teamcode.RobotCore.BehaviorTree.Node.TaskNodes.TaskSequence.TaskSequence;
import org.firstinspires.ftc.teamcode.RobotCore.BehaviorTree.States;
import org.firstinspires.ftc.teamcode.RobotCore.TaskUtils.StandartArgs;
import org.firstinspires.ftc.teamcode.RobotCore.TaskUtils.Tasks.OrdinaryTask;

import java.util.Stack;

public class PrepareToElement extends TaskSequence implements ConstsTeleskope {
    public PrepareToElement(Robot robot, LinearOpMode lin) {
        super(robot);
        this.lin = lin;

        OrdinaryTask upCaptured = new OrdinaryTask(robot.setZahvat,
                new StandartArgs.captureArgs(HANG_POS_FLIP, CLOSE_POS_HOOK, 0.3),
                OrdinaryTask.taskStartMode.START_AFTER_PREVIOUS);

        taskToDo.push(upCaptured);

        countTasks = 1;
    }

    public Stack<OrdinaryTask> taskToDo = new Stack<>();

    public int countTasks;

    public int completCount = 0;

    public LinearOpMode lin;

    @Override
    public void stopProgramm() {
        stop = true;
    }

    @Override
    public void programm() {
        if (!taskToDo.isEmpty()) robot.taskManager.addTaskToStack(taskToDo.peek());

        if (!taskToDo.isEmpty()){
            while ((taskToDo.peek().state == States.TODO || taskToDo.peek().state == States.RUNNING) && !lin.isStopRequested() && lin.opModeIsActive()) {
                robot.taskManager.permanentlyExecute();
                lin.telemetry.update();
            }
        }

        if (!taskToDo.isEmpty() && taskToDo.peek().state == States.SUCCESS) {
            taskToDo.pop();//Смахиваем выполненую задачу с вершины Стэка
            completCount++;
        }

        if (!taskToDo.isEmpty() && taskToDo.peek().state == States.FAILURE) {
            nodeState = States.FAILURE;
        }

        if (completCount == countTasks) {
            nodeState = States.SUCCESS;
        }
    }
}
