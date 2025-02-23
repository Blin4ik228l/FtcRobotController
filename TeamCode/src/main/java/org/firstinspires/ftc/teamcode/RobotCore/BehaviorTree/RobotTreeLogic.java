package org.firstinspires.ftc.teamcode.RobotCore.BehaviorTree;

import org.firstinspires.ftc.teamcode.RobotCore.TaskUtils.TaskManager;

public class RobotTreeLogic {
    //TODO: Дописать в целом логику
    private TaskManager taskManager;

    private double progressBar;
    public RobotTreeLogic(TaskManager taskManager){this.taskManager = taskManager;
    }

    public void setProgressBarFromTask(double progress){
        progressBar = progress;
    }

    public void Root(){

    }

}

