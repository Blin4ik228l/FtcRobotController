package org.firstinspires.ftc.teamcode.OpModes.Autonoms;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.OpModes.Robot;
import org.firstinspires.ftc.teamcode.RobotCore.RobotUtils.RobotAlliance;
import org.firstinspires.ftc.teamcode.RobotCore.RobotUtils.RobotMode;
import org.firstinspires.ftc.teamcode.RobotCore.TaskUtils.StandartArgs;
import org.firstinspires.ftc.teamcode.RobotCore.TaskUtils.Task;
import org.firstinspires.ftc.teamcode.RobotCore.TaskUtils.TaskHandler;
import org.firstinspires.ftc.teamcode.RobotCore.TaskUtils.TaskManager;
import org.firstinspires.ftc.teamcode.RobotCore.Utils.Position;

@Autonomous(name = "RedRight", group = "Red", preselectTeleOp = "TeleOpRed")
public class AutoRedRight extends LinearOpMode {
    Robot robot;
    Position posForDrive1 = new Position(50, 50 , 0);
    Position posForDrive2 = new Position( 100, 0, 0);
    Position posForDrive3 = new Position( 100, 0, 0);
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(RobotMode.AUTO, RobotAlliance.RED, this);
        robot.init();
        robot.odometry.setGlobalPosition(new Position(0,0,0));
        Task taskDrive1 = new Task(robot.driveToPosition, new StandartArgs.driveStandartArgs(posForDrive1), 5, Task.taskStartMode.START_AFTER_PREVIOUS);
        waitForStart();
        if(!isStopRequested()) {
            robot.taskManager.addTask(taskDrive1);
            robot.taskManager.start();
        }
    }


}
