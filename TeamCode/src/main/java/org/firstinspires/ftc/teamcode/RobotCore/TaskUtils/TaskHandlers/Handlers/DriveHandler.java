package org.firstinspires.ftc.teamcode.RobotCore.TaskUtils.TaskHandlers.Handlers;

import org.firstinspires.ftc.teamcode.RobotCore.RobotStatus.OtherStates.RobotStatus;
import org.firstinspires.ftc.teamcode.RobotCore.RobotStatus.OtherStates.RobotStatusInDrive;
import org.firstinspires.ftc.teamcode.RobotCore.TaskUtils.TaskHandlers.TaskHandlerOrdinal;

import java.util.Deque;

public interface DriveHandler extends TaskHandlerOrdinal {
    Deque<RobotStatusInDrive>[] statusInDrive();
    RobotStatus statusRobot();
}
