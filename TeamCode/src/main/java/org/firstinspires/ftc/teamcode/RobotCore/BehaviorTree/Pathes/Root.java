package org.firstinspires.ftc.teamcode.RobotCore.BehaviorTree.Pathes;

import org.firstinspires.ftc.teamcode.OpModes.Robot;
import org.firstinspires.ftc.teamcode.RobotCore.TaskUtils.Task;
import org.firstinspires.ftc.teamcode.RobotCore.TaskUtils.TaskManager;

public class Root {
    private Sequence sequence;
    private TaskManager taskManager;
    private Robot robot;

    //Начало программы
    public Root(TaskManager taskManager, Robot robot){
        this.taskManager = taskManager;
        this.robot = robot;

        sequence = new Sequence(robot);

        programmHandler();
    }

    public void programmHandler(){
        //Отбирать задачи в действии

        if(taskManager.getExecutingDeque().getFirst().startMode == Task.taskStartMode.START_WITH_PREVIOUS){
            sequence.addAction(taskManager.getExecutingDeque().getFirst(), taskManager.getExecutingDeque().iterator().next());
        }else{
            sequence.addAction(taskManager.getExecutingDeque().getFirst());
        }

        for (int i = 0; i < sequence.getFailureTasks().length; i++) {
            //Убираем с отсортированой очереди задачу, которая провалилась со статусом HOTCAKE
            if(sequence.getFailureTasks()[i] != null) {
                taskManager.getExecutingDeque().removeFirst();
                taskManager.addTask(sequence.getFailureTasks()[i]);
            }
        }

    }


}
