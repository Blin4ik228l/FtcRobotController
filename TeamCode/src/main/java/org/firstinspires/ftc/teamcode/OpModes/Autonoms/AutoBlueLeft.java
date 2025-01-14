package org.firstinspires.ftc.teamcode.OpModes.Autonoms;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.OpModes.Robot;
import org.firstinspires.ftc.teamcode.RobotCore.RobotUtils.RobotAlliance;
import org.firstinspires.ftc.teamcode.RobotCore.RobotUtils.RobotMode;
import org.firstinspires.ftc.teamcode.RobotCore.TaskUtils.StandartArgs;
import org.firstinspires.ftc.teamcode.RobotCore.TaskUtils.Task;
import org.firstinspires.ftc.teamcode.RobotCore.Utils.CONSTSTELESKOPE;
import org.firstinspires.ftc.teamcode.RobotCore.Utils.Position;

@Autonomous(name = "BlueLeft", group = "Blue", preselectTeleOp = "BlueMeow")
public class AutoBlueLeft extends LinearOpMode {
    Robot robot;
    Position pos1 = new Position(65, -80 , Math.toRadians(0));
    Position pos2 = new Position( 47, -80, Math.toRadians(0));
    Position pos3 = new Position( 60, 60, Math.toRadians(135));
    Position pos4 = new Position( 30, 40,Math.toRadians(-225) );
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(RobotMode.AUTO,RobotAlliance.BLUE, this);
        robot.init();
        robot.odometry.setGlobalPosition(new Position(0,0,0));
        Task zahvat1 = new Task(robot.setZahvat, new StandartArgs.zahvatStandartArgs(CONSTSTELESKOPE.HANG_POS_FLIP, CONSTSTELESKOPE.CLOSE_POS_HOOK),5, Task.taskStartMode.START_AFTER_PREVIOUS);

        Task drive1 = new Task(robot.driveToPosition, new StandartArgs.driveStandartArgs(pos1, 190), 5, Task.taskStartMode.START_AFTER_PREVIOUS);

        Task upTele1 = new Task(robot.setTeleskopePos, new StandartArgs.teleskopeStandartArgs(CONSTSTELESKOPE.UP_TUBE_HEIGHT, 0.20, 1), 5, Task.taskStartMode.START_AFTER_PREVIOUS);

        Task drive2 = new Task(robot.driveToPosition, new StandartArgs.driveStandartArgs(pos2, 30), 5, Task.taskStartMode.START_AFTER_PREVIOUS);

        Task upTele2 = new Task(robot.driveToPosition,new StandartArgs.teleskopeStandartArgs(CONSTSTELESKOPE.TAKING_HEIGHT, CONSTSTELESKOPE.CLOSE_POS_HORIZONTAL, 0.6), 5, Task.taskStartMode.START_AFTER_PREVIOUS);

        robot.taskManager.addTask(zahvat1);
        robot.taskManager.addTask(drive1);
        robot.taskManager.addTask(upTele1);
        robot.taskManager.addTask(drive2);

        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            robot.taskManager.forAuto();
        }
    }
}
