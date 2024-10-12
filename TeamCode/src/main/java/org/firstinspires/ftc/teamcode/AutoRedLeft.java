package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "RedLeft", group = "Red", preselectTeleOp = "TeleOpRed")
public class AutoRedLeft extends LinearOpMode {
ROBOT my_robot = new ROBOT();
    @Override
    public void runOpMode() throws InterruptedException {
        Args.driveArgs args = new Args.driveArgs(new Position(12,16,0), 20);
        ROBOT.Task new_task = new ROBOT.Task(ROBOT.taskType.DRIVE_TO_POSITION, ROBOT.taskRunMode.START_AFTER_PREVIOUS, args);
        my_robot.addTask(new_task);
    }
}
