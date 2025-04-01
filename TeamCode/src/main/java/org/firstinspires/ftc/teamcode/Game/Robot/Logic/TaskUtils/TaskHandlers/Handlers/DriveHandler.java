package org.firstinspires.ftc.teamcode.Game.Robot.Logic.TaskUtils.TaskHandlers.Handlers;

import org.firstinspires.ftc.teamcode.Game.Robot.Logic.TaskUtils.TaskHandlers.TaskHandlerOrdinal;
import org.firstinspires.ftc.teamcode.Game.Robot.Logic.RobotModulesStatus.OtherStatuses.RobotStatusInDrive;
import org.firstinspires.ftc.teamcode.Game.Robot.Logic.RobotModulesStatus.RobotModuleStatus;

import java.util.Deque;

public interface DriveHandler extends TaskHandlerOrdinal {
    Deque<RobotStatusInDrive>[] statusInDrive();
    RobotModuleStatus statusRobot();
}
