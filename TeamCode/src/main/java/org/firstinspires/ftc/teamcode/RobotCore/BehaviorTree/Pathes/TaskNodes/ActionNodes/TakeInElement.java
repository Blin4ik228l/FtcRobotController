package org.firstinspires.ftc.teamcode.RobotCore.BehaviorTree.Pathes.TaskNodes.ActionNodes;

import org.firstinspires.ftc.teamcode.Consts.ConstsTeleskope;
import org.firstinspires.ftc.teamcode.RobotCore.BehaviorTree.Pathes.TaskNodes.TaskSequence;
import org.firstinspires.ftc.teamcode.RobotCore.BehaviorTree.States;
import org.firstinspires.ftc.teamcode.RobotCore.TaskUtils.StandartArgs;
import org.firstinspires.ftc.teamcode.RobotCore.TaskUtils.Tasks.OrdinaryTask;

public class TakeInElement extends TaskSequence implements ConstsTeleskope {
    //Это заранее написаная подпрограмма для взятия игрового элемента
    //Состоит в структуре дерева

    public TakeInElement(){
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
    }
    @Override
    public void tickMe() {
        programm();

        if(completCount == 3) nodeState = States.SUCCESS;
    }
}
