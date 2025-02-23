package org.firstinspires.ftc.teamcode.RobotCore.BehaviorTree.Pathes;

import org.firstinspires.ftc.teamcode.OpModes.Robot;
import org.firstinspires.ftc.teamcode.RobotCore.TaskUtils.Task;

class Sequence {
    //Подпрограмма отвечающая за выполнение конечных действий
    private Selector selector;

    private Task[] runningTasks;
    private Task[] failureTasks;

    public Sequence(Robot robot) {
       selector = new Selector(robot);
    }

    public void addAction(Task...tasks){
        // tasks - это массив[]
        runningTasks = tasks;
        failureTasks = new Task[tasks.length];

        actionHandler();
    }

    public Task[] getFailureTasks() {
        return failureTasks;
    }

    public void actionHandler(){
        for (int i = 0; i < runningTasks.length; i++) {
            if(selector.resultHandler(runningTasks[i]) == Task.States.FAILURE){
                failureTasks[i] = runningTasks[i];
                failureTasks[i].startMode = Task.taskStartMode.HOTCAKE;
            }
        }
    }
}
