package org.firstinspires.ftc.teamcode.RobotCore.TaskUtils.Tasks.VarietiesTasks;

import org.firstinspires.ftc.teamcode.RobotCore.TaskUtils.StandartArgs;
import org.firstinspires.ftc.teamcode.RobotCore.TaskUtils.TaskHandlers.Handlers.ZahvatHandler;
import org.firstinspires.ftc.teamcode.RobotCore.TaskUtils.Tasks.OrdinaryTask;

public class ZahvatTask extends OrdinaryTask {
    public ZahvatTask(ZahvatHandler zahvatHandler, StandartArgs.captureArgs args, OrdinaryTask.taskStartMode startMode) {
        super(zahvatHandler, args, startMode);
        this.zahvatHandler = zahvatHandler;
        this.args = args;
    }
    public ZahvatHandler zahvatHandler;
    public StandartArgs.captureArgs args;
}
