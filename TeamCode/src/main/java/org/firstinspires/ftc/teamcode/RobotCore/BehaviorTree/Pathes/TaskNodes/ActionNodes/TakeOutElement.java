package org.firstinspires.ftc.teamcode.RobotCore.BehaviorTree.Pathes.TaskNodes.ActionNodes;

import org.firstinspires.ftc.teamcode.Consts.ConstsTeleskope;
import org.firstinspires.ftc.teamcode.RobotCore.BehaviorTree.Pathes.TaskNodes.TaskSequence;
import org.firstinspires.ftc.teamcode.RobotCore.BehaviorTree.States;
import org.firstinspires.ftc.teamcode.RobotCore.TaskUtils.StandartArgs;
import org.firstinspires.ftc.teamcode.RobotCore.TaskUtils.Tasks.OrdinaryTask;

public class TakeOutElement extends TaskSequence implements ConstsTeleskope {
    //Это заранее написаная подпрограмма для размещения игрового элемента
    //Состоит в структуре дерева
    public TakeOutElement(){
        OrdinaryTask prepareCapture = new OrdinaryTask(robot.setZahvat,
                new StandartArgs.captureArgs(TAKE_POS_FLIP, CLOSE_POS_HOOK),
                OrdinaryTask.taskStartMode.START_AFTER_PREVIOUS);

        OrdinaryTask throwAway = new OrdinaryTask(robot.setZahvat,
                new StandartArgs.captureArgs(TAKE_POS_FLIP, OPEN_POS_HOOK),
                OrdinaryTask.taskStartMode.START_AFTER_PREVIOUS);

        OrdinaryTask upUnCaptured = new OrdinaryTask(robot.setZahvat,
                new StandartArgs.captureArgs(HANG_POS_FLIP, CLOSE_POS_HOOK),
                OrdinaryTask.taskStartMode.START_AFTER_PREVIOUS);

        taskToDo.push(upUnCaptured);
        taskToDo.push(throwAway);
        taskToDo.push(prepareCapture);
    }

    @Override
    public void tickMe() {
        programm();

        if(completCount == 3) nodeState = States.SUCCESS;
    }

}
