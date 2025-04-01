package org.firstinspires.ftc.teamcode.Game.Robot.Logic.TaskUtils.TaskHandlers;

import org.firstinspires.ftc.teamcode.Game.Robot.Logic.TaskUtils.StandartArgs;
import org.firstinspires.ftc.teamcode.Game.Robot.Logic.TaskUtils.TaskManager;

public interface TaskHandlerOrdinal {
    int init(final TaskManager thisTaskManager, StandartArgs _args);
    int execute(final TaskManager thisTaskManager, StandartArgs _args);
}
