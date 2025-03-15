package org.firstinspires.ftc.teamcode.RobotCore.BehaviorTree.Node.TaskNodes.TaskSequence.TaskSequences;

import org.firstinspires.ftc.teamcode.Consts.ConstsTeleskope;
import org.firstinspires.ftc.teamcode.OpModes.Robot;
import org.firstinspires.ftc.teamcode.RobotCore.BehaviorTree.Node.TaskNodes.TaskSequence.TaskSequence;
import org.firstinspires.ftc.teamcode.RobotCore.BehaviorTree.States;
import org.firstinspires.ftc.teamcode.RobotCore.TaskUtils.StandartArgs;
import org.firstinspires.ftc.teamcode.RobotCore.TaskUtils.Tasks.OrdinaryTask;

import java.util.Stack;

public class TakeInElementSeq extends TaskSequence implements ConstsTeleskope {
    //Это заранее написаная подпрограмма для взятия игрового элемента
    //Состоит в структуре дерева

    public TakeInElementSeq(Robot robot){
        super(robot);
        OrdinaryTask prepareCapture = new OrdinaryTask(robot.setZahvat,
                new StandartArgs.captureArgs(TAKE_POS_FLIP, OPEN_POS_HOOK),
                OrdinaryTask.taskStartMode.START_AFTER_PREVIOUS);

        OrdinaryTask capture = new OrdinaryTask(robot.setZahvat,
                new StandartArgs.captureArgs(TAKE_POS_FLIP, CLOSE_POS_HOOK),
                OrdinaryTask.taskStartMode.START_AFTER_PREVIOUS);

        OrdinaryTask upCaptured = new OrdinaryTask(robot.setZahvat,
                new StandartArgs.captureArgs(HANG_POS_FLIP, CLOSE_POS_HOOK),
                OrdinaryTask.taskStartMode.START_AFTER_PREVIOUS);

        taskToDo.push(upCaptured);
        taskToDo.push(capture);
        taskToDo.push(prepareCapture);

        countTasks = 3;
    }

    public Stack<OrdinaryTask> taskToDo = new Stack<>();

    public int countTasks;

    public int completCount = 0;


    @Override
    public void stopProgramm() {
        stop = true;
    }

    @Override
    public void programm() {
        if (!taskToDo.isEmpty() && nodeState != States.FAILURE) robot.taskManager.addTaskToStack(taskToDo.peek());

        if (!taskToDo.isEmpty()) {

            robot.taskManager.permanentlyExecute();

            if (taskToDo.peek().state == States.SUCCESS) {
                taskToDo.pop();//Смахиваем выполненую задачу с вершины Стэка
                completCount++;
            }
            if (completCount == countTasks) {
                nodeState = States.SUCCESS;
            }

            if (taskToDo.peek().state == States.FAILURE) {
                nodeState = States.FAILURE;
            }
        }
    }


}
