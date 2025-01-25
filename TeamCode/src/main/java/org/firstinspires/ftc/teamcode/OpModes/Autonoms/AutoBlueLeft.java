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

import java.util.Objects;

@Autonomous(name = "BlueLeft", group = "Blue", preselectTeleOp = "BlueMeow")
public class AutoBlueLeft extends LinearOpMode implements CONSTSTELESKOPE, CONSTS, REWARDSFORACTIONS {
    Robot robot;
//    Position pos1 = new Position(70, -83 , 0);
//    Position pos2 = new Position( 78, -83, 0);
//    Position pos3 = new Position( 65, -80, 90);
//    Position pos4 = new Position( 75, 15,45);

    Position pos1 = new Position(30, 40,135);//25, 45, 135
    Position pos2 = new Position(30, 32,0);//25, 33, 0
    Position pos3 = new Position(30, 63, 0);//25, 60, 0
    Position pos4 = new Position(30, 40,-225);//25, 60, -225
    Position pos5 = new Position(150, 0, 0); //25, -240, 90 МОЖЕТ БЫТЬ ОТСТУП 5 СМ ОТ КАРЗИНЫ
    Position pos6 = new Position(150, -40, 0);
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(RobotMode.AUTO,RobotAlliance.BLUE, this);
        robot.init();
        robot.odometry.setGlobalPosition(new Position(0,0,0));// 2 клетка от карзины

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

        Task downTeleWhile = new Task(robot.doWhile, new StandartArgs.doWhile(-1), NOTHING, Task.taskStartMode.START_AFTER_PREVIOUS);

        Task upTele232 = new Task(robot.setTeleskopePos, new StandartArgs.teleskopeStandartArgs(TAKING_HEIGHT, CLOSE_POS_HORIZONTAL, 1, "Sfsf") ,0,Task.taskStartMode.START_AFTER_PREVIOUS);
        Task driveToBasket =
                new Task(robot.driveToPosition, new StandartArgs.driveStandartArgs(pos1, 80), NOTHING, Task.taskStartMode.START_AFTER_PREVIOUS);

        Task upTele =
                new Task(robot.setTeleskopePos, new StandartArgs.teleskopeStandartArgs(BUSKET_HEIGHT, CLOSE_POS_HORIZONTAL_AUTO, 1,  "Up tele to upper busket and Move horizontal to" + " " + CLOSE_POS_HORIZONTAL_AUTO), NOTHING, Task.taskStartMode.START_AFTER_PREVIOUS );

        Task moveHorizontal =
                new Task(robot.setTeleskopePos, new StandartArgs.teleskopeStandartArgs(BUSKET_HEIGHT, 0.22, 0.4, "Move horizontal to"+ " " + 0.22), NOTHING, Task.taskStartMode.START_AFTER_PREVIOUS);

        Task sleep1 =
                new Task(robot.robotSleep, new StandartArgs.robotSleep(500),NOTHING,Task.taskStartMode.START_AFTER_PREVIOUS);

        Task takeOutSample =
                new Task(robot.setZahvat, new StandartArgs.zahvatStandartArgs(THROW_POS_FLIP, OPEN_POS_HOOK), NOTHING, Task.taskStartMode.START_AFTER_PREVIOUS);

        Task hangSample =
                new Task(robot.setZahvat, new StandartArgs.zahvatStandartArgs(HANG_POS_FLIP, OPEN_POS_HOOK), NOTHING, Task.taskStartMode.START_AFTER_PREVIOUS);

        Task moveHorizontal2 =
                new Task(robot.setTeleskopePos, new StandartArgs.teleskopeStandartArgs(BUSKET_HEIGHT, CLOSE_POS_HORIZONTAL, 1, "Move horizontal to"+ " " + CLOSE_POS_HOOK), NOTHING, Task.taskStartMode.START_AFTER_PREVIOUS);

        Task sleep2 =
                new Task(robot.robotSleep, new StandartArgs.robotSleep(500),NOTHING,Task.taskStartMode.START_AFTER_PREVIOUS);

        Task downTele =
                new Task(robot.setTeleskopePos, new StandartArgs.teleskopeStandartArgs(TAKING_HEIGHT, CLOSE_POS_HORIZONTAL, 1, "Down tele to land"), NOTHING, Task.taskStartMode.START_AFTER_PREVIOUS);

        Task RotateTo1Sample =
                new Task(robot.driveToPosition, new StandartArgs.driveStandartArgs(pos2, 90), NOTHING, Task.taskStartMode.START_WITH_PREVIOUS);

        Task moveHorizontal3 =
                new Task(robot.setTeleskopePos, new StandartArgs.teleskopeStandartArgs(TAKING_HEIGHT, 0.2, 1, "Move horizontal to"+ " " + OPEN_POS_HORIZONTAL), NOTHING, Task.taskStartMode.START_AFTER_PREVIOUS);

        Task sleep3 =
                new Task(robot.robotSleep, new StandartArgs.robotSleep(1000), NOTHING, Task.taskStartMode.START_AFTER_PREVIOUS);

        Task taking =
                new Task(robot.setZahvat, new StandartArgs.zahvatStandartArgs(TAKE_POS_FLIP2, OPEN_POS_HOOK), NOTHING, Task.taskStartMode.START_AFTER_PREVIOUS);

        Task getting =
                new Task(robot.setZahvat, new StandartArgs.zahvatStandartArgs(TAKE_POS_FLIP, CLOSE_POS_HOOK), NOTHING, Task.taskStartMode.START_AFTER_PREVIOUS);

        Task upping =
                new Task(robot.setZahvat, new StandartArgs.zahvatStandartArgs(HANG_POS_FLIP, CLOSE_POS_HOOK), NOTHING, Task.taskStartMode.START_AFTER_PREVIOUS);

//        Task RotateTo3Sample =
//                new Task(robot.driveToPosition, new StandartArgs.driveStandartArgs(pos4, 120), NOTHING, Task.taskStartMode.START_AFTER_PREVIOUS);
        Task RotateTo2Sample =
                new Task(robot.driveToPosition, new StandartArgs.driveStandartArgs(pos3, 80), NOTHING, Task.taskStartMode.START_AFTER_PREVIOUS);

        Task moveHorizontal4 =
                new Task(robot.setTeleskopePos, new StandartArgs.teleskopeStandartArgs(TAKING_HEIGHT, 0.25, 1, "Move horizontal to"+ " " + OPEN_POS_HORIZONTAL), NOTHING, Task.taskStartMode.START_AFTER_PREVIOUS);

        Task moveHorizontal5 =
                new Task(robot.setTeleskopePos, new StandartArgs.teleskopeStandartArgs(TAKING_HEIGHT,  CLOSE_POS_HORIZONTAL, 1, "Move horizontal to"+ " " + OPEN_POS_HORIZONTAL), NOTHING, Task.taskStartMode.START_AFTER_PREVIOUS);

        Task driveToBasket2 =
                new Task(robot.driveToPosition, new StandartArgs.driveStandartArgs(pos4, 30), NOTHING, Task.taskStartMode.START_AFTER_PREVIOUS);

        Task driveTParking =
                new Task(robot.driveToPosition, new StandartArgs.driveStandartArgs(pos5, 100), NOTHING, Task.taskStartMode.START_AFTER_PREVIOUS);

        Task driveToBasket5 =
                new Task(robot.driveToPosition, new StandartArgs.driveStandartArgs(pos1, 30), NOTHING, Task.taskStartMode.START_AFTER_PREVIOUS);

        Task driveTParking2 =
                new Task(robot.driveToPosition, new StandartArgs.driveStandartArgs(pos6, 100), NOTHING, Task.taskStartMode.START_AFTER_PREVIOUS);

        robot.taskManager.addTask(downTeleWhile);
        robot.taskManager.addTask(upTele232);
        robot.taskManager.addTask(driveToBasket);
        robot.taskManager.addTask(upTele);
        robot.taskManager.addTask(moveHorizontal);
        robot.taskManager.addTask(sleep1);
        robot.taskManager.addTask(takeOutSample);
        robot.taskManager.addTask(hangSample);
        robot.taskManager.addTask(moveHorizontal2);
        robot.taskManager.addTask(sleep2);
        robot.taskManager.addTask(downTele);

        robot.taskManager.addTask(sleep2);
//
        robot.taskManager.addTask(RotateTo1Sample);
        robot.taskManager.addTask(moveHorizontal4);
        robot.taskManager.addTask(taking);
        robot.taskManager.addTask(sleep3);
        robot.taskManager.addTask(getting);
        robot.taskManager.addTask(sleep2);
        robot.taskManager.addTask(upping);
        robot.taskManager.addTask(sleep2);
        robot.taskManager.addTask(moveHorizontal5);

        robot.taskManager.addTask(sleep3);
//
        robot.taskManager.addTask(driveToBasket5);
        robot.taskManager.addTask(upTele);
        robot.taskManager.addTask(moveHorizontal);
        robot.taskManager.addTask(sleep2);
        robot.taskManager.addTask(takeOutSample);
        robot.taskManager.addTask(hangSample);
        robot.taskManager.addTask(sleep2);
        robot.taskManager.addTask(moveHorizontal2);
        robot.taskManager.addTask(sleep2);
        robot.taskManager.addTask(downTele);

//        robot.taskManager.addTask(RotateTo2Sample);
//        robot.taskManager.addTask(moveHorizontal3);
//        robot.taskManager.addTask(taking);
//        robot.taskManager.addTask(sleep2);
//        robot.taskManager.addTask(getting);
//        robot.taskManager.addTask(sleep2);
//        robot.taskManager.addTask(upping);
//        robot.taskManager.addTask(sleep2);
//        robot.taskManager.addTask(moveHorizontal2);
//        robot.taskManager.addTask(upTele);
//        robot.taskManager.addTask(driveToBasket2);
//
//
//        robot.taskManager.addTask(moveHorizontal);
//        robot.taskManager.addTask(sleep2);
//        robot.taskManager.addTask(takeOutSample);
//        robot.taskManager.addTask(hangSample);
//        robot.taskManager.addTask(moveHorizontal2);
//        robot.taskManager.addTask(sleep2);
//        robot.taskManager.addTask(downTele);
//
        robot.taskManager.addTask(sleep3);

        robot.taskManager.addTask(driveTParking);
        robot.taskManager.addTask(driveTParking2);

//        robot.taskManager.addTask(driveToHunging);
//        robot.taskManager.addTask(HungTele);
//        robot.taskManager.addTask(CammingToHung);
//        robot.taskManager.addTask(downTele);



        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            robot.taskManager.forAuto();
        }
    }
}
