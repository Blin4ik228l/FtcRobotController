package org.firstinspires.ftc.teamcode.Game.Robot.Logic.TaskUtils.Tasks.VarietiesTasks;

import org.firstinspires.ftc.teamcode.Game.Robot.Logic.TaskUtils.StandartArgs;
import org.firstinspires.ftc.teamcode.Game.Robot.Logic.TaskUtils.TaskHandlers.Handlers.TeleskopeHandler;
import org.firstinspires.ftc.teamcode.Game.Robot.Logic.TaskUtils.Tasks.OrdinaryTask;

public class TeleskopeTask extends OrdinaryTask {
    public TeleskopeTask(TeleskopeHandler teleskopeHandler, StandartArgs.verticalArgs args, OrdinaryTask.taskStartMode startMode) {
        super(teleskopeHandler, args,  startMode);
        this.teleskopeHandler = teleskopeHandler;
        this.args = args;
    }
    public TeleskopeHandler teleskopeHandler;
    public StandartArgs.verticalArgs args;
}
