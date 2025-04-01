package org.firstinspires.ftc.teamcode.Game.Robot.Logic.BehaviorTree.Node.TaskNodes.TaskSequence.TaskSequences;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Game.Robot.Logic.BehaviorTree.Node.TaskNodes.TaskSequence.TaskSequence;
import org.firstinspires.ftc.teamcode.Game.Robot.Logic.BehaviorTree.States;
import org.firstinspires.ftc.teamcode.Utils.Consts.ConstsTeleskope;
import org.firstinspires.ftc.teamcode.Game.Programms.Robot;
import org.firstinspires.ftc.teamcode.Game.Robot.Logic.TaskUtils.StandartArgs;
import org.firstinspires.ftc.teamcode.Game.Robot.Logic.TaskUtils.Tasks.OrdinaryTask;

import java.util.Stack;

public class TakeInElementSeq extends TaskSequence implements ConstsTeleskope {
    //Это заранее написаная подпрограмма для взятия игрового элемента
    //Состоит в структуре дерева

    public TakeInElementSeq(Robot robot, LinearOpMode lin){
        super(robot);
        this.lin = lin;
        OrdinaryTask prepareCapture = new OrdinaryTask(robot.setZahvat,
                new StandartArgs.captureArgs(TAKE_POS_FLIP, OPEN_POS_HOOK, 0.4),
                OrdinaryTask.taskStartMode.START_AFTER_PREVIOUS);

        OrdinaryTask capture = new OrdinaryTask(robot.setZahvat,
                new StandartArgs.captureArgs(TAKE_POS_FLIP, CLOSE_POS_HOOK, 0.15),
                OrdinaryTask.taskStartMode.START_AFTER_PREVIOUS);

        OrdinaryTask upCaptured = new OrdinaryTask(robot.setZahvat,
                new StandartArgs.captureArgs(HANG_POS_FLIP, CLOSE_POS_HOOK, 0.15),
                OrdinaryTask.taskStartMode.START_AFTER_PREVIOUS);


        taskToDo.push(upCaptured);

        taskToDo.push(capture);

        taskToDo.push(prepareCapture);

        countTasks = 3;
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
        lin.telemetry.addData("completCount",completCount);
        lin.telemetry.update();
    }
}
