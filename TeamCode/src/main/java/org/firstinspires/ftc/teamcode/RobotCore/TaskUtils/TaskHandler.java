package org.firstinspires.ftc.teamcode.RobotCore.TaskUtils;

public interface TaskHandler {
    int init(final TaskManager thisTaskManager, StandartArgs _args);
    int execute(final TaskManager thisTaskManager, StandartArgs _args, Task task);
}
