package org.firstinspires.ftc.teamcode.RobotCore.TaskUtils.TaskHandlers.Handlers;

import org.firstinspires.ftc.teamcode.RobotCore.RobotModulesStatus.OtherStatuses.RobotStatusInDrive;
import org.firstinspires.ftc.teamcode.RobotCore.RobotModulesStatus.RobotModuleStatus;
import org.firstinspires.ftc.teamcode.RobotCore.TaskUtils.TaskHandlers.TaskHandlerOrdinal;

import java.util.Deque;

public interface DriveHandler extends TaskHandlerOrdinal {
    Deque<RobotStatusInDrive>[] statusInDrive();
    RobotModuleStatus statusRobot();
}
