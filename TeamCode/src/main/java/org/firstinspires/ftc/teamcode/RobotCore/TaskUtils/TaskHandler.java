package org.firstinspires.ftc.teamcode.RobotCore.TaskUtils;

import org.firstinspires.ftc.teamcode.RobotCore.RobotStatus.RobotStatus;
import org.firstinspires.ftc.teamcode.RobotCore.RobotStatus.RobotStatusInDrive;

import java.util.Deque;

public interface TaskHandler {
    int init(final TaskManager thisTaskManager, StandartArgs _args);
    int execute(final TaskManager thisTaskManager, StandartArgs _args);

    Deque<RobotStatusInDrive>[] statusInDrive();
    RobotStatus statusRobot();
}
