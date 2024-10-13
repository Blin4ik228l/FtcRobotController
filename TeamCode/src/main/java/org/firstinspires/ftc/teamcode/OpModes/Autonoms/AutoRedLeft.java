package org.firstinspires.ftc.teamcode.OpModes.Autonoms;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotCore.RobotSubsystems.RobotCore;
import org.firstinspires.ftc.teamcode.RobotCore.TaskUtils.StdArgs;
import org.firstinspires.ftc.teamcode.RobotCore.TaskUtils.Task;
import org.firstinspires.ftc.teamcode.RobotCore.TaskUtils.TaskExecMode;

@Autonomous(name = "RedLeft", group = "Red", preselectTeleOp = "TeleOpRed")
public class AutoRedLeft extends LinearOpMode {

    RobotCore my_robot = new RobotCore(TaskExecMode.AUTO);

    @Override
    public void runOpMode() throws InterruptedException {
        my_robot.taskManager.addTask(new Task(my_robot.example, new StdArgs(), Task.taskRunMode.START_AFTER_PREVIOUS));
        my_robot.taskManager.start();
    }
}
