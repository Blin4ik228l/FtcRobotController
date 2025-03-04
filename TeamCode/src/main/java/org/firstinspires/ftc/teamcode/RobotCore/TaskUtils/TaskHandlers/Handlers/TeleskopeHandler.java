package org.firstinspires.ftc.teamcode.RobotCore.TaskUtils.TaskHandlers.Handlers;

import org.firstinspires.ftc.teamcode.RobotCore.RobotModulesStatus.OtherStatuses.TeleskopeStatusInMoving;
import org.firstinspires.ftc.teamcode.RobotCore.TaskUtils.TaskHandlers.TaskHandlerOrdinal;

import java.util.Deque;

public interface TeleskopeHandler extends TaskHandlerOrdinal {
    Deque<TeleskopeStatusInMoving>[] status();
}
