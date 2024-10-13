package org.firstinspires.ftc.teamcode.OpModes.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotCore.RobotSubsystems.RobotCore;
import org.firstinspires.ftc.teamcode.RobotCore.TaskUtils.StdArgs;
import org.firstinspires.ftc.teamcode.RobotCore.TaskUtils.Task;
import org.firstinspires.ftc.teamcode.RobotCore.TaskUtils.TaskExecMode;
import org.firstinspires.ftc.teamcode.Utils.Position;


@TeleOp
public class TeleopTest extends OpMode {

    RobotCore robot = new RobotCore(TaskExecMode.TELEOP);

    @Override
    public void init() {
        robot.init();
        robot.odometry.setGlobalPosition(new Position(0,0,0));

        Task newtask = new Task(robot.example, new StdArgs(), Task.taskRunMode.START_WITH_PREVIOUS);
        robot.taskManager.addTask(newtask);
    }

    @Override
    public void loop() {

    }
}
