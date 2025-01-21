package org.firstinspires.ftc.teamcode.OpModes.Autonoms;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.OpModes.Robot;
import org.firstinspires.ftc.teamcode.RobotCore.RobotUtils.REWARDSFORACTIONS;
import org.firstinspires.ftc.teamcode.RobotCore.RobotUtils.RobotAlliance;
import org.firstinspires.ftc.teamcode.RobotCore.RobotUtils.RobotMode;
import org.firstinspires.ftc.teamcode.RobotCore.TaskUtils.StandartArgs;
import org.firstinspires.ftc.teamcode.RobotCore.TaskUtils.Task;
import org.firstinspires.ftc.teamcode.RobotCore.Utils.CONSTS;
import org.firstinspires.ftc.teamcode.RobotCore.Utils.CONSTSTELESKOPE;
import org.firstinspires.ftc.teamcode.RobotCore.Utils.Position;

@Autonomous(name = "BlueRight", group = "Blue", preselectTeleOp = "TeleOpBlue")
public class AutoBlueRight extends OpMode implements CONSTSTELESKOPE, CONSTS, REWARDSFORACTIONS {
    Robot robot;
    Position pos1 = new Position(180,10,135);
    Position pos11 = new Position(180,10,135);
    Position pos2 = new Position(180,10,-8);
    Position pos3 = new Position(180,10,10);

    /**
     *  Метод вызывается один раз при нажатии INIT
     */
    @Override
    public void init() {
        robot = new Robot(RobotMode.AUTO, RobotAlliance.BLUE, this);

        robot.init();
        robot.odometry.setGlobalPosition(new Position(0,0,0)); // 3 клетка от стенки

//        Task driveTask1 = new Task(robot.driveToPosition, new StandartArgs.driveStandartArgs(position1), 5, Task.taskStartMode.START_AFTER_PREVIOUS, "");
//        robot.taskManager.addTask(driveTask1);

        Task driveToBasket =
                new Task(robot.driveToPosition, new StandartArgs.driveStandartArgs(pos1, 80), NOTHING, Task.taskStartMode.START_AFTER_PREVIOUS);

        Task upTele =
                new Task(robot.setTeleskopePos, new StandartArgs.teleskopeStandartArgs(BUSKET_HEIGHT, CLOSE_POS_HORIZONTAL_AUTO, 1,  "Up tele to upper busket and Move horizontal to" + " " + CLOSE_POS_HORIZONTAL_AUTO), NOTHING, Task.taskStartMode.START_AFTER_PREVIOUS );

        Task moveHorizontal =
                new Task(robot.setTeleskopePos, new StandartArgs.teleskopeStandartArgs(BUSKET_HEIGHT, 0.18, 0.4, "Move horizontal to"+ " " + 0.22), NOTHING, Task.taskStartMode.START_AFTER_PREVIOUS);

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
                new Task(robot.driveToPosition, new StandartArgs.driveStandartArgs(pos2, 80), NOTHING, Task.taskStartMode.START_AFTER_PREVIOUS);

        Task moveHorizontal3 =
                new Task(robot.setTeleskopePos, new StandartArgs.teleskopeStandartArgs(TAKING_HEIGHT, 0.2, 1, "Move horizontal to"+ " " + OPEN_POS_HORIZONTAL), NOTHING, Task.taskStartMode.START_AFTER_PREVIOUS);

        Task sleep3 =
                new Task(robot.robotSleep, new StandartArgs.robotSleep(1000), NOTHING, Task.taskStartMode.START_AFTER_PREVIOUS);

        Task taking =
                new Task(robot.setZahvat, new StandartArgs.zahvatStandartArgs(TAKE_POS_FLIP, OPEN_POS_HOOK), NOTHING, Task.taskStartMode.START_AFTER_PREVIOUS);

        Task getting =
                new Task(robot.setZahvat, new StandartArgs.zahvatStandartArgs(TAKE_POS_FLIP, CLOSE_POS_HOOK), NOTHING, Task.taskStartMode.START_AFTER_PREVIOUS);

//        Task upping =
//                new Task(robot.setZahvat, new StandartArgs.zahvatStandartArgs(HANG_POS_FLIP, CLOSE_POS_HOOK), NOTHING, Task.taskStartMode.START_AFTER_PREVIOUS);
//
//        Task RotateTo3Sample =
//                new Task(robot.driveToPosition, new StandartArgs.driveStandartArgs(pos4, 120), NOTHING, Task.taskStartMode.START_AFTER_PREVIOUS);
        Task RotateTo2Sample =
                new Task(robot.driveToPosition, new StandartArgs.driveStandartArgs(pos3, 120), NOTHING, Task.taskStartMode.START_AFTER_PREVIOUS);



        robot.taskManager.addTask(driveToBasket);
        robot.taskManager.addTask(upTele);
        robot.taskManager.addTask(moveHorizontal);
        robot.taskManager.addTask(sleep1);
        robot.taskManager.addTask(takeOutSample);
        robot.taskManager.addTask(hangSample);
        robot.taskManager.addTask(moveHorizontal2);
        robot.taskManager.addTask(sleep2);
        robot.taskManager.addTask(downTele);



        robot.taskManager.addTask(RotateTo1Sample);
        robot.taskManager.addTask(moveHorizontal3);
        robot.taskManager.addTask(taking);
        robot.taskManager.addTask(sleep3);
        robot.taskManager.addTask(getting);
        robot.taskManager.addTask(moveHorizontal2);

        robot.taskManager.addTask(driveToBasket);
        robot.taskManager.addTask(upTele);
        robot.taskManager.addTask(moveHorizontal);
        robot.taskManager.addTask(sleep1);
        robot.taskManager.addTask(takeOutSample);
        robot.taskManager.addTask(hangSample);
        robot.taskManager.addTask(moveHorizontal2);
        robot.taskManager.addTask(sleep2);
        robot.taskManager.addTask(downTele);



        robot.taskManager.addTask(RotateTo2Sample);
        robot.taskManager.addTask(moveHorizontal3);
        robot.taskManager.addTask(taking);
        robot.taskManager.addTask(sleep3);
        robot.taskManager.addTask(getting);
        robot.taskManager.addTask(moveHorizontal2);

        robot.taskManager.addTask(driveToBasket);
        robot.taskManager.addTask(upTele);
        robot.taskManager.addTask(moveHorizontal);
        robot.taskManager.addTask(sleep1);
        robot.taskManager.addTask(takeOutSample);
        robot.taskManager.addTask(hangSample);
        robot.taskManager.addTask(moveHorizontal2);
        robot.taskManager.addTask(sleep2);
        robot.taskManager.addTask(downTele);
    }

    /**
     *  Метод крутится в цикле, ожидая нажатия START
     */
    @Override
    public void init_loop() {
    }

    /**
     *  Метод вызывается один раз при нажатии кнопки START
     */
    @Override
    public void start() {

    }

    /**
     *  Метод крутится в цикле после нажатия START
     */
    @Override
    public void loop() {
        robot.taskManager.start();
    }

    /**
     *  Метод вызывается один раз при нажатии STOP
     */
    @Override
    public void stop() {
        robot.robotMode = RobotMode.STOP;
    }

}
