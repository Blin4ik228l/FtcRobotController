package org.firstinspires.ftc.teamcode.RobotCore.TaskUtils.Tasks.VarietiesTasks;

import org.firstinspires.ftc.teamcode.RobotCore.TaskUtils.StandartArgs;
import org.firstinspires.ftc.teamcode.RobotCore.TaskUtils.TaskHandlers.Handlers.DriveHandler;
import org.firstinspires.ftc.teamcode.RobotCore.TaskUtils.Tasks.OrdinaryTask;

public class DriveTask extends OrdinaryTask {

    public DriveTask(DriveHandler driveHandler, StandartArgs.driveStandartArgs args, OrdinaryTask.taskStartMode startMode){
        super(driveHandler,args, startMode);
        this.driveHandler = driveHandler;
        this.args = args;
    }

    public DriveHandler driveHandler;
    public StandartArgs.driveStandartArgs args;
}
