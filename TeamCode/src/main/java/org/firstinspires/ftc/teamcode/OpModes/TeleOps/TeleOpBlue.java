package org.firstinspires.ftc.teamcode.OpModes.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.OpModes.Robot;
import org.firstinspires.ftc.teamcode.RobotCore.RobotUtils.RobotAlliance;
import org.firstinspires.ftc.teamcode.RobotCore.RobotUtils.RobotMode;
import org.firstinspires.ftc.teamcode.RobotCore.TaskUtils.StandartArgs;
import org.firstinspires.ftc.teamcode.RobotCore.TaskUtils.Task;
import org.firstinspires.ftc.teamcode.RobotCore.Utils.Position;


@TeleOp(name = "BlueMeow", group = "Blue")
public class TeleOpBlue extends OpMode {

    Robot robot;

    /**
     *  Метод вызывается один раз при нажатии INIT
     */
    @Override
    public void init() {
        robot = new Robot(RobotMode.TELEOP, RobotAlliance.BLUE, this);

        robot.odometry.setGlobalPosition(new Position(robot.odometry.getGlobalPosition().x,robot.odometry.getGlobalPosition().y,robot.odometry.getGlobalPosition().heading));
        robot.init();

        Task newtask = new Task(robot.example, new StandartArgs(), 5, Task.taskStartMode.START_WITH_PREVIOUS);
        robot.taskManager.addTask(newtask);
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
    }

    /**
     *  Метод крутится в цикле после нажатия START
     */
    @Override
    public void loop() {
        robot.teleopPl1();
        robot.teleopPl2();
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
