package org.firstinspires.ftc.teamcode.RobotCore.TaskUtils.TaskHandlers.Handlers;

import org.firstinspires.ftc.teamcode.RobotCore.RobotStatus.RobotStatus;
import org.firstinspires.ftc.teamcode.RobotCore.RobotStatus.RobotStatusInDrive;
import org.firstinspires.ftc.teamcode.RobotCore.TaskUtils.TaskHandlers.TaskHandlerOrdinal;

import java.util.Deque;

public interface DriveHandler extends TaskHandlerOrdinal {
    Deque<RobotStatusInDrive>[] statusInDrive();
    RobotStatus statusRobot();
}
