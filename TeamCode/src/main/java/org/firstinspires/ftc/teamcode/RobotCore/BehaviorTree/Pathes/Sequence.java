package org.firstinspires.ftc.teamcode.RobotCore.BehaviorTree.Pathes;

import org.firstinspires.ftc.teamcode.OpModes.Robot;
import org.firstinspires.ftc.teamcode.RobotCore.TaskUtils.Tasks.OrdinaryTask;

class Sequence {
    //Подпрограмма отвечающая за выполнение конечных действий
    private Selector selector;

    private OrdinaryTask[] runningTasks;
    private OrdinaryTask[] failureTasks;

    public Sequence(Robot robot) {
       selector = new Selector(robot);
    }

    public void addAction(OrdinaryTask...tasks){
        // tasks - это массив[]
        runningTasks = tasks;
        failureTasks = new OrdinaryTask[tasks.length];

        actionHandler();
    }

    public OrdinaryTask[] getFailureTasks() {
        return failureTasks;
    }

    public void actionHandler(){
        for (int i = 0; i < runningTasks.length; i++) {
            if(selector.resultHandler(runningTasks[i]) == OrdinaryTask.States.FAILURE){
                failureTasks[i] = runningTasks[i];
                failureTasks[i].startMode = OrdinaryTask.taskStartMode.HOTCAKE;
            }
        }
    }
}
