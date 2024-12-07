package org.firstinspires.ftc.teamcode.OpModes.Autonoms;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.OpModes.Robot;
import org.firstinspires.ftc.teamcode.RobotCore.RobotUtils.RobotAlliance;
import org.firstinspires.ftc.teamcode.RobotCore.RobotUtils.RobotMode;
import org.firstinspires.ftc.teamcode.RobotCore.TaskUtils.StandartArgs;
import org.firstinspires.ftc.teamcode.RobotCore.TaskUtils.Task;
import org.firstinspires.ftc.teamcode.RobotCore.Utils.Position;

@Autonomous(name = "BlueLeft", group = "Blue", preselectTeleOp = "TeleOpBlue")
public class AutoBlueLeft extends LinearOpMode {
    Robot robot;
    Position pos1 = new Position(70, -80 , Math.toRadians(0));
    Position pos2 = new Position( 60, 60, Math.toRadians(0));
    Position pos3 = new Position( 60, 60, Math.toRadians(135));
    Position pos4 = new Position( 30, 40,Math.toRadians(-225) );
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(RobotMode.AUTO,RobotAlliance.BLUE, this);
        robot.init();
        robot.odometry.setGlobalPosition(new Position(0,0,0));
        Task task1 = new Task(robot.driveToPosition, new StandartArgs.driveStandartArgs(pos1, 190), 5, Task.taskStartMode.START_AFTER_PREVIOUS);
        robot.taskManager.addTask(task1);
        Task task2 = new Task(robot.driveToPosition, new StandartArgs.driveStandartArgs(pos2), 5, Task.taskStartMode.START_AFTER_PREVIOUS);
        robot.taskManager.addTask(task2);
        Task task3 = new Task(robot.driveToPosition, new StandartArgs.driveStandartArgs(pos3), 5, Task.taskStartMode.START_AFTER_PREVIOUS);
        robot.taskManager.addTask(task3);
        Task task4 = new Task(robot.driveToPosition, new StandartArgs.driveStandartArgs(pos4), 5, Task.taskStartMode.START_AFTER_PREVIOUS);
//        robot.taskManager.addTask(task4);
        waitForStart();
        while (opModeIsActive()) {
            robot.taskManager.start();
        }
    }
}
