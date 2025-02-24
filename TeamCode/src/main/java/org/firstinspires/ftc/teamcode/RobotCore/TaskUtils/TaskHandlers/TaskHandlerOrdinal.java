package org.firstinspires.ftc.teamcode.RobotCore.TaskUtils.TaskHandlers;

import org.firstinspires.ftc.teamcode.RobotCore.TaskUtils.StandartArgs;
import org.firstinspires.ftc.teamcode.RobotCore.TaskUtils.TaskManager;

public interface TaskHandlerOrdinal {
    int init(final TaskManager thisTaskManager, StandartArgs _args);
    int execute(final TaskManager thisTaskManager, StandartArgs _args);
}
