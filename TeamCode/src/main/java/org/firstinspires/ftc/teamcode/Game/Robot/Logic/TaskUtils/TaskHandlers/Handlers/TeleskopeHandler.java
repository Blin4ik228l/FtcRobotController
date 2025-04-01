package org.firstinspires.ftc.teamcode.Game.Robot.Logic.TaskUtils.TaskHandlers.Handlers;

import org.firstinspires.ftc.teamcode.Game.Robot.Logic.RobotModulesStatus.OtherStatuses.TeleskopeStatusInMoving;
import org.firstinspires.ftc.teamcode.Game.Robot.Logic.TaskUtils.TaskHandlers.TaskHandlerOrdinal;

import java.util.Deque;

public interface TeleskopeHandler extends TaskHandlerOrdinal {
    Deque<TeleskopeStatusInMoving>[] status();
}
