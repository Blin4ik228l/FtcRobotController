package org.firstinspires.ftc.teamcode.RobotCore.TaskUtils.TaskHandlers.Handlers;

import org.firstinspires.ftc.teamcode.RobotCore.RobotStatus.TeleskopeStatusInAction;
import org.firstinspires.ftc.teamcode.RobotCore.TaskUtils.TaskHandlers.TaskHandlerOrdinal;

import java.util.Deque;

public interface TeleskopeHandler extends TaskHandlerOrdinal {
    Deque<TeleskopeStatusInAction>[] status();
}
