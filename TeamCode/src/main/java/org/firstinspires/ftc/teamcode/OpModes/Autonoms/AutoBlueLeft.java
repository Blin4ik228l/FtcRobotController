package org.firstinspires.ftc.teamcode.OpModes.Autonoms;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.OpModes.Robot;
import org.firstinspires.ftc.teamcode.RobotCore.RobotUtils.REWARDSFORACTIONS;
import org.firstinspires.ftc.teamcode.RobotCore.RobotUtils.RobotAlliance;
import org.firstinspires.ftc.teamcode.RobotCore.RobotUtils.RobotMode;
import org.firstinspires.ftc.teamcode.RobotCore.TaskUtils.StandartArgs;
import org.firstinspires.ftc.teamcode.RobotCore.TaskUtils.Task;
import org.firstinspires.ftc.teamcode.RobotCore.Utils.CONSTSTELESKOPE;
import org.firstinspires.ftc.teamcode.RobotCore.Utils.Position;

@Autonomous(name = "BlueLeft", group = "Blue", preselectTeleOp = "BlueMeow")
public class AutoBlueLeft extends LinearOpMode implements CONSTSTELESKOPE, REWARDSFORACTIONS {
    Robot robot;
//    Position pos1 = new Position(70, -83 , 0);
//    Position pos2 = new Position( 78, -83, 0);
//    Position pos3 = new Position( 65, -80, 90);
//    Position pos4 = new Position( 75, 15,45);

    Position pos1 = new Position( 20, 45,135);
    Position pos2 = new Position( 20, 45,30);
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(RobotMode.AUTO,RobotAlliance.BLUE, this);
        robot.init();
        robot.odometry.setGlobalPosition(new Position(0,0,0));

//        Task zahvat1 = new Task(robot.setZahvat, new StandartArgs.zahvatStandartArgs(CONSTSTELESKOPE.HANG_POS_FLIP, CONSTSTELESKOPE.CLOSE_POS_HOOK), REWARDSFORACTIONS.NOTHING, Task.taskStartMode.START_AFTER_PREVIOUS, "Take sample");

//        Task drive1 = new Task(robot.driveToPosition, new StandartArgs.driveStandartArgs(pos1, 120), REWARDSFORACTIONS.NOTHING, Task.taskStartMode.START_AFTER_PREVIOUS, "Go to upper pipe");

//        Task upTele1 = new Task(robot.setTeleskopePos, new StandartArgs.teleskopeStandartArgs(CONSTSTELESKOPE.UP_TUBE_HEIGHT, CONSTSTELESKOPE.CLOSE_POS_HORIZONTAL_AUTO, 1), REWARDSFORACTIONS.NOTHING, Task.taskStartMode.START_AFTER_PREVIOUS, "Up tele to upper pipe");
//
//        Task drive2 = new Task(robot.driveToPosition, new StandartArgs.driveStandartArgs(pos2, CONSTS.MIN_LINEAR_SPEED), 5, Task.taskStartMode.START_AFTER_PREVIOUS, "Slowly clossing");
//
//        Task downTele1 = new Task(robot.setTeleskopePos,new StandartArgs.teleskopeStandartArgs(30, CONSTSTELESKOPE.CLOSE_POS_HORIZONTAL_AUTO, 0.4), 5, Task.taskStartMode.START_AFTER_PREVIOUS, "Down tele for hanging on pipe");
//
//        Task zahvat2 = new Task(robot.setZahvat, new StandartArgs.zahvatStandartArgs(CONSTSTELESKOPE.HANG_POS_FLIP, CONSTSTELESKOPE.OPEN_POS_HOOK), REWARDSFORACTIONS.NOTHING, Task.taskStartMode.START_AFTER_PREVIOUS, "Hang sample");
//
//        Task drive3 = new Task(robot.driveToPosition, new StandartArgs.driveStandartArgs(pos3, CONSTS.MIN_LINEAR_SPEED), 5, Task.taskStartMode.START_WITH_PREVIOUS, "Move back");
//
//        Task downTele2 = new Task(robot.setTeleskopePos,new StandartArgs.teleskopeStandartArgs(CONSTSTELESKOPE.TAKING_HEIGHT, CONSTSTELESKOPE.CLOSE_POS_HORIZONTAL, 1), 5, Task.taskStartMode.START_WITH_PREVIOUS, "Down tele for taking from land sample1");
//
//        Task drive4 = new Task(robot.driveToPosition, new StandartArgs.driveStandartArgs(pos4, 140), 5, Task.taskStartMode.START_AFTER_PREVIOUS, "Move for sample1");
//
//        Task zahvat3 = new Task(robot.setZahvat, new StandartArgs.zahvatStandartArgs(CONSTSTELESKOPE.TAKE_POS_FLIP, CONSTSTELESKOPE.OPEN_POS_HOOK), REWARDSFORACTIONS.NOTHING, Task.taskStartMode.START_AFTER_PREVIOUS, "Take sample");
//
//        Task zahvat4 = new Task(robot.setZahvat, new StandartArgs.zahvatStandartArgs(CONSTSTELESKOPE.TAKE_POS_FLIP, CONSTSTELESKOPE.CLOSE_POS_HOOK), REWARDSFORACTIONS.NOTHING, Task.taskStartMode.START_AFTER_PREVIOUS, "Take sample");



        Task drive1 =
                new Task(robot.driveToPosition, new StandartArgs.driveStandartArgs(pos1, 120), NOTHING, Task.taskStartMode.START_AFTER_PREVIOUS,
                        "Go to upper busket");

        Task upTele1 =
                new Task(robot.setTeleskopePos, new StandartArgs.teleskopeStandartArgs(BUSKET_HEIGHT, 0.18, 1), NOTHING, Task.taskStartMode.START_AFTER_PREVIOUS,
                        "Up tele to upper busket");

        Task zahvat2 =
                new Task(robot.setZahvat, new StandartArgs.zahvatStandartArgs(HANG_POS_FLIP, OPEN_POS_HOOK), NOTHING, Task.taskStartMode.START_AFTER_PREVIOUS,
                        "Hang sample");

        Task closeTele1 =
                new Task(robot.setTeleskopePos, new StandartArgs.teleskopeStandartArgs(BUSKET_HEIGHT, CLOSE_POS_HORIZONTAL, 1), NOTHING, Task.taskStartMode.START_AFTER_PREVIOUS,
                        "Up tele to upper pipe");

        Task downTele1 =
                new Task(robot.setTeleskopePos, new StandartArgs.teleskopeStandartArgs(TAKING_HEIGHT, CLOSE_POS_HORIZONTAL, 1), NOTHING, Task.taskStartMode.START_AFTER_PREVIOUS,
                        "Up tele to upper pipe");

        Task drive2 =
                new Task(robot.driveToPosition, new StandartArgs.driveStandartArgs(pos2, 120), NOTHING, Task.taskStartMode.START_AFTER_PREVIOUS,
                        "Go to upper busket");


        robot.taskManager.addTask(drive1);
        robot.taskManager.addTask(upTele1);
        robot.taskManager.addTask(zahvat2);
        robot.taskManager.addTask(closeTele1);
        robot.taskManager.addTask(downTele1);
        robot.taskManager.addTask(drive2);

//        robot.taskManager.addTask(drive2);
//        robot.taskManager.addTask(downTele1);


//        robot.taskManager.addTask(drive3);
//        robot.taskManager.addTask(downTele2);
//        robot.taskManager.addTask(drive4);
//        robot.taskManager.addTask(zahvat3);

        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            robot.taskManager.forAuto();
        }
    }
}
