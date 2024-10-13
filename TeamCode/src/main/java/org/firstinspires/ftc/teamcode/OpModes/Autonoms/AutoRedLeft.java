package org.firstinspires.ftc.teamcode.OpModes.Autonoms;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot.TaskHandler.Args;
import org.firstinspires.ftc.teamcode.Robot.TaskHandler.Tasks;
import org.firstinspires.ftc.teamcode.Utils.Position;
import org.firstinspires.ftc.teamcode.Robot.ROBOT;
import org.firstinspires.ftc.teamcode.Utils.Vector2;

@Autonomous(name = "RedLeft", group = "Red", preselectTeleOp = "TeleOpRed")
public class AutoRedLeft extends LinearOpMode {
ROBOT my_robot = new ROBOT();
    @Override
    public void runOpMode() throws InterruptedException {
        Args.driveArgs args = new Args.driveArgs(new Position(new Vector2(1,1), 1), 1, 0);
        Tasks new_task = new Tasks(Tasks.taskType.DRIVE_TO_POSITION, Tasks.taskRunMode.START_AFTER_PREVIOUS, args);
        my_robot.addTask(new_task);
    }
}
