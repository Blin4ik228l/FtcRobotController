package org.firstinspires.ftc.teamcode.Game.Robot.Logic.TaskUtils.Tasks.VarietiesTasks;

import org.firstinspires.ftc.teamcode.Game.Robot.Logic.TaskUtils.StandartArgs;
import org.firstinspires.ftc.teamcode.Game.Robot.Logic.TaskUtils.TaskHandlers.Handlers.DriveHandler;
import org.firstinspires.ftc.teamcode.Game.Robot.Logic.TaskUtils.Tasks.OrdinaryTask;

public class DriveTask extends OrdinaryTask {

    public DriveTask(DriveHandler driveHandler, StandartArgs.driveArgs args, OrdinaryTask.taskStartMode startMode){
        super(driveHandler,args, startMode);
        this.driveHandler = driveHandler;
        this.args = args;
    }

    public DriveHandler driveHandler;
    public StandartArgs.driveArgs args;
}
