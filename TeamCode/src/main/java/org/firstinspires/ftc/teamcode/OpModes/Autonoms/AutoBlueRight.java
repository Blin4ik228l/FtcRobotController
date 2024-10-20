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

@Autonomous(name = "RightBlue", group = "Blue", preselectTeleOp = "TeleOpBlue")
public class AutoBlueRight extends OpMode {
    Robot robot;
    Position position1 = new Position(50,50,30);
    Position position2 = new Position(100,212,0);
    Position position3 = new Position(0,0,0);
    /**
     *  Метод вызывается один раз при нажатии INIT
     */
    @Override
    public void init() {
        robot = new Robot(RobotMode.TELEOP, RobotAlliance.BLUE, this);

        robot.odometry.setGlobalPosition(new Position(0,0,0));
        robot.init();

        Task driveTask1 = new Task(robot.driveToPosition, new StandartArgs.driveStandartArgs(position1), 5, Task.taskStartMode.START_WITH_PREVIOUS);
        robot.taskManager.addTask(driveTask1);

        Task driveTask2 = new Task(robot.driveToPosition, new StandartArgs.driveStandartArgs(position2), 5, Task.taskStartMode.START_WITH_PREVIOUS);
        robot.taskManager.addTask(driveTask2);

        Task driveTask3 = new Task(robot.driveToPosition, new StandartArgs.driveStandartArgs(position3), 5, Task.taskStartMode.START_WITH_PREVIOUS);
        robot.taskManager.addTask(driveTask3);
    }

    /**
     *  Метод крутится в цикле, ожидая нажатия START
     */
    @Override
    public void init_loop() {
        super.init_loop();
    }

    /**
     *  Метод вызывается один раз при нажатии кнопки START
     */
    @Override
    public void start() {
        super.start();
        robot.taskManager.start();
    }

    /**
     *  Метод крутится в цикле после нажатия START
     */
    @Override
    public void loop() {

    }

    /**
     *  Метод вызывается один раз при нажатии STOP
     */
    @Override
    public void stop() {
        super.stop();
        robot.messageTelemetry.killTelemetry();
    }

}
