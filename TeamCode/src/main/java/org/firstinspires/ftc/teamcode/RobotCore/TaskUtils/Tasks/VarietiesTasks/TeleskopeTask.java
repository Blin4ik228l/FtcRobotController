package org.firstinspires.ftc.teamcode.RobotCore.TaskUtils.Tasks.VarietiesTasks;

import org.firstinspires.ftc.teamcode.RobotCore.TaskUtils.StandartArgs;
import org.firstinspires.ftc.teamcode.RobotCore.TaskUtils.TaskHandlers.Handlers.TeleskopeHandler;
import org.firstinspires.ftc.teamcode.RobotCore.TaskUtils.Tasks.OrdinaryTask;

public class TeleskopeTask extends OrdinaryTask {
    public TeleskopeTask(TeleskopeHandler teleskopeHandler, StandartArgs.teleskopeArgs args, OrdinaryTask.taskStartMode startMode) {
        super(teleskopeHandler, args,  startMode);
        this.teleskopeHandler = teleskopeHandler;
        this.args = args;
    }
    public TeleskopeHandler teleskopeHandler;
    public StandartArgs.teleskopeArgs args;
}
