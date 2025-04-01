package org.firstinspires.ftc.teamcode.Game.Programms.Autonomuses.Classes.Red;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Game.Programms.Autonomuses.LinearOpModeModified;
import org.firstinspires.ftc.teamcode.Utils.Consts.Consts;
import org.firstinspires.ftc.teamcode.Utils.Consts.ConstsTeleskope;
import org.firstinspires.ftc.teamcode.Utils.Consts.RewardsForActions;
import org.firstinspires.ftc.teamcode.Game.Programms.Robot;
import org.firstinspires.ftc.teamcode.Game.Robot.Logic.RobotUtils.RobotAlliance;
import org.firstinspires.ftc.teamcode.Game.Robot.Logic.RobotUtils.RobotMode;
import org.firstinspires.ftc.teamcode.Game.Robot.Logic.Utils.Position;

@Autonomous(name = "RedRight", group = "Red", preselectTeleOp = "TeleOpRed")
public class AutoRedRight extends LinearOpModeModified implements ConstsTeleskope, Consts, RewardsForActions {
    Position pos1 = new Position(180,10,135);
    Position pos11 = new Position(180,10,135);
    Position pos2 = new Position(180,10,-8);
    Position pos3 = new Position(180,10,10);
    @Override
    public void runOpMode() throws InterruptedException {

        r = new Robot(RobotMode.AUTO, RobotAlliance.RED, this, new Position(0,0,0));
        r.init();

//
//        Task driveToBasket =
//                new Task(robot.driveToPosition, new StandartArgs.driveStandartArgs(pos1, 80), NOTHING, Task.taskStartMode.START_AFTER_PREVIOUS);
//
//        Task upTele =
//                new Task(robot.setTeleskopePos, new StandartArgs.teleskopeStandartArgs(BUSKET_HEIGHT, CLOSE_POS_HORIZONTAL_AUTO, 1,  "Up tele to upper busket and Move horizontal to" + " " + CLOSE_POS_HORIZONTAL_AUTO), NOTHING, Task.taskStartMode.START_AFTER_PREVIOUS );
//
//        Task moveHorizontal =
//                new Task(robot.setTeleskopePos, new StandartArgs.teleskopeStandartArgs(BUSKET_HEIGHT, 0.18, 0.4, "Move horizontal to"+ " " + 0.22), NOTHING, Task.taskStartMode.START_AFTER_PREVIOUS);
//
//        Task sleep1 =
//                new Task(robot.robotSleep, new StandartArgs.robotSleep(500),NOTHING,Task.taskStartMode.START_AFTER_PREVIOUS);
//
//        Task takeOutSample =
//                new Task(robot.setZahvat, new StandartArgs.zahvatStandartArgs(THROW_POS_FLIP, OPEN_POS_HOOK), NOTHING, Task.taskStartMode.START_AFTER_PREVIOUS);
//
//        Task hangSample =
//                new Task(robot.setZahvat, new StandartArgs.zahvatStandartArgs(HANG_POS_FLIP, OPEN_POS_HOOK), NOTHING, Task.taskStartMode.START_AFTER_PREVIOUS);
//
//        Task moveHorizontal2 =
//                new Task(robot.setTeleskopePos, new StandartArgs.teleskopeStandartArgs(BUSKET_HEIGHT, CLOSE_POS_HORIZONTAL, 1, "Move horizontal to"+ " " + CLOSE_POS_HOOK), NOTHING, Task.taskStartMode.START_AFTER_PREVIOUS);
//
//        Task sleep2 =
//                new Task(robot.robotSleep, new StandartArgs.robotSleep(500),NOTHING,Task.taskStartMode.START_AFTER_PREVIOUS);
//
//        Task downTele =
//                new Task(robot.setTeleskopePos, new StandartArgs.teleskopeStandartArgs(TAKING_HEIGHT, CLOSE_POS_HORIZONTAL, 1, "Down tele to land"), NOTHING, Task.taskStartMode.START_AFTER_PREVIOUS);
//
//        Task RotateTo1Sample =
//                new Task(robot.driveToPosition, new StandartArgs.driveStandartArgs(pos2, 80), NOTHING, Task.taskStartMode.START_AFTER_PREVIOUS);
//
//        Task moveHorizontal3 =
//                new Task(robot.setTeleskopePos, new StandartArgs.teleskopeStandartArgs(TAKING_HEIGHT, 0.2, 1, "Move horizontal to"+ " " + OPEN_POS_HORIZONTAL), NOTHING, Task.taskStartMode.START_AFTER_PREVIOUS);
//
//        Task sleep3 =
//                new Task(robot.robotSleep, new StandartArgs.robotSleep(1500), NOTHING, Task.taskStartMode.START_AFTER_PREVIOUS);
//
//        Task taking =
//                new Task(robot.setZahvat, new StandartArgs.zahvatStandartArgs(TAKE_POS_FLIP, OPEN_POS_HOOK), NOTHING, Task.taskStartMode.START_AFTER_PREVIOUS);
//
//        Task getting =
//                new Task(robot.setZahvat, new StandartArgs.zahvatStandartArgs(TAKE_POS_FLIP, CLOSE_POS_HOOK), NOTHING, Task.taskStartMode.START_AFTER_PREVIOUS);
//        Task RotateTo2Sample =
//                new Task(robot.driveToPosition, new StandartArgs.driveStandartArgs(pos3, 120), NOTHING, Task.taskStartMode.START_AFTER_PREVIOUS);
//
//
//
//        robot.taskManager.addTask(driveToBasket);
//        robot.taskManager.addTask(upTele);
//        robot.taskManager.addTask(moveHorizontal);
//        robot.taskManager.addTask(sleep1);
//        robot.taskManager.addTask(takeOutSample);
//        robot.taskManager.addTask(hangSample);
//        robot.taskManager.addTask(moveHorizontal2);
//        robot.taskManager.addTask(sleep2);
//        robot.taskManager.addTask(downTele);
//
//
//        robot.taskManager.addTask(RotateTo1Sample);
//        robot.taskManager.addTask(moveHorizontal3);
//        robot.taskManager.addTask(taking);
//        robot.taskManager.addTask(sleep3);
//        robot.taskManager.addTask(getting);
//        robot.taskManager.addTask(sleep3);
//        robot.taskManager.addTask(moveHorizontal2);
//
//
//
//        robot.taskManager.addTask(driveToBasket);
//        robot.taskManager.addTask(upTele);
//        robot.taskManager.addTask(moveHorizontal);
//        robot.taskManager.addTask(sleep1);
//        robot.taskManager.addTask(takeOutSample);
//        robot.taskManager.addTask(hangSample);
//        robot.taskManager.addTask(moveHorizontal2);
//        robot.taskManager.addTask(sleep2);
//        robot.taskManager.addTask(downTele);
//
//
//
//        robot.taskManager.addTask(RotateTo2Sample);
//        robot.taskManager.addTask(moveHorizontal3);
//        robot.taskManager.addTask(taking);
//        robot.taskManager.addTask(sleep3);
//        robot.taskManager.addTask(getting);
//        robot.taskManager.addTask(moveHorizontal2);
//
//        robot.taskManager.addTask(driveToBasket);
//        robot.taskManager.addTask(upTele);
//        robot.taskManager.addTask(moveHorizontal);
//        robot.taskManager.addTask(sleep1);
//        robot.taskManager.addTask(takeOutSample);
//        robot.taskManager.addTask(hangSample);
//        robot.taskManager.addTask(moveHorizontal2);
//        robot.taskManager.addTask(sleep2);
//        robot.taskManager.addTask(downTele);

    }
}
