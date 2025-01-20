package org.firstinspires.ftc.teamcode.OpModes.Autonoms;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.OpModes.Robot;
import org.firstinspires.ftc.teamcode.RobotCore.RobotUtils.REWARDSFORACTIONS;
import org.firstinspires.ftc.teamcode.RobotCore.RobotUtils.RobotAlliance;
import org.firstinspires.ftc.teamcode.RobotCore.RobotUtils.RobotMode;
import org.firstinspires.ftc.teamcode.RobotCore.TaskUtils.StandartArgs;
import org.firstinspires.ftc.teamcode.RobotCore.TaskUtils.Task;
import org.firstinspires.ftc.teamcode.RobotCore.Utils.CONSTS;
import org.firstinspires.ftc.teamcode.RobotCore.Utils.CONSTSTELESKOPE;
import org.firstinspires.ftc.teamcode.RobotCore.Utils.Position;

@Autonomous(name = "BlueLeft", group = "Blue", preselectTeleOp = "BlueMeow")
public class AutoBlueLeft extends LinearOpMode {
    Robot robot;
    Position pos1 = new Position(75, -83 , 0);
    Position pos2 = new Position( 82, -83, 0);
    Position pos3 = new Position( 65, -80, 90);
    Position pos4 = new Position( 65, 30,90);
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(RobotMode.AUTO,RobotAlliance.BLUE, this);
        robot.init();
        robot.odometry.setGlobalPosition(new Position(0,0,0));

//        Task zahvat1 = new Task(robot.setZahvat, new StandartArgs.zahvatStandartArgs(CONSTSTELESKOPE.HANG_POS_FLIP, CONSTSTELESKOPE.CLOSE_POS_HOOK), REWARDSFORACTIONS.NOTHING, Task.taskStartMode.START_AFTER_PREVIOUS, "Take sample");

        Task drive1 = new Task(robot.driveToPosition, new StandartArgs.driveStandartArgs(pos1, 140), REWARDSFORACTIONS.NOTHING, Task.taskStartMode.START_AFTER_PREVIOUS, "Go to upper pipe");

        Task upTele1 = new Task(robot.setTeleskopePos, new StandartArgs.teleskopeStandartArgs(CONSTSTELESKOPE.UP_TUBE_HEIGHT, CONSTSTELESKOPE.CLOSE_POS_HORIZONTAL_AUTO, 1), REWARDSFORACTIONS.NOTHING, Task.taskStartMode.START_AFTER_PREVIOUS, "Up tele to upper pipe");

        Task drive2 = new Task(robot.driveToPosition, new StandartArgs.driveStandartArgs(pos2, CONSTS.MIN_LINEAR_SPEED), 5, Task.taskStartMode.START_AFTER_PREVIOUS, "Slowly clossing");

        Task downTele1 = new Task(robot.setTeleskopePos,new StandartArgs.teleskopeStandartArgs(40, CONSTSTELESKOPE.CLOSE_POS_HORIZONTAL_AUTO, 0.6), 5, Task.taskStartMode.START_AFTER_PREVIOUS, "Down tele for hanging on pipe");

        Task zahvat2 = new Task(robot.setZahvat, new StandartArgs.zahvatStandartArgs(CONSTSTELESKOPE.HANG_POS_FLIP, CONSTSTELESKOPE.OPEN_POS_HOOK), REWARDSFORACTIONS.NOTHING, Task.taskStartMode.START_AFTER_PREVIOUS, "Hang sample");

        Task drive3 = new Task(robot.driveToPosition, new StandartArgs.driveStandartArgs(pos3, CONSTS.MIN_LINEAR_SPEED), 5, Task.taskStartMode.START_WITH_PREVIOUS, "Move back");

        Task downTele2 = new Task(robot.setTeleskopePos,new StandartArgs.teleskopeStandartArgs(CONSTSTELESKOPE.TAKING_HEIGHT, CONSTSTELESKOPE.CLOSE_POS_HORIZONTAL, 1), 5, Task.taskStartMode.START_WITH_PREVIOUS, "Down tele for taking from land sample1");

        Task drive4 = new Task(robot.driveToPosition, new StandartArgs.driveStandartArgs(pos4, CONSTS.MIN_LINEAR_SPEED), 5, Task.taskStartMode.START_AFTER_PREVIOUS, "Move for sample1");

//        robot.taskManager.addTask(zahvat1);
        robot.taskManager.addTask(drive1);
        robot.taskManager.addTask(upTele1);
        robot.taskManager.addTask(drive2);
        robot.taskManager.addTask(downTele1);
        robot.taskManager.addTask(zahvat2);
        robot.taskManager.addTask(drive3);
        robot.taskManager.addTask(downTele2);
        robot.taskManager.addTask(drive4);

        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            robot.taskManager.forAuto();
        }
    }
}
