package org.firstinspires.ftc.teamcode.OpModes.TeleOps;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.OpModes.Robot;
import org.firstinspires.ftc.teamcode.RobotCore.RobotUtils.RobotAlliance;
import org.firstinspires.ftc.teamcode.RobotCore.RobotUtils.RobotMode;
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

        robot.odometry.setGlobalPosition(new Position(robot.odometry.getGlobalPosition().getX(),robot.odometry.getGlobalPosition().getY(),robot.odometry.getGlobalPosition().getHeading()));
        robot.init();

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
//        robot.taskManager.start();

        robot.checkJoysticks();
//        robot.updateColors();


        robot.teleopPl1();
        robot.teleopPl2();

        robot.telemetry();
        robot.dataDisplayer.update();
    }

    /**
     *  Метод вызывается один раз при нажатии STOP
     */
    @Override
    public void stop() {
        robot.robotMode = RobotMode.STOP;
    }
}
