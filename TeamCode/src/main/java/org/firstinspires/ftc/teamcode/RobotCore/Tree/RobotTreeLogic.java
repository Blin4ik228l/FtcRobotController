package org.firstinspires.ftc.teamcode.RobotCore.Tree;

import org.firstinspires.ftc.teamcode.RobotCore.TaskUtils.TaskManager;

public class RobotTreeLogic {
    private TaskManager taskManager;

    private double progressBar;
    public RobotTreeLogic(TaskManager taskManager){
        this.taskManager = taskManager;
    }

    public void setProgressBarFromTask(double progress){
        progressBar = progress;
    }

    public void wather(){

    }
}
