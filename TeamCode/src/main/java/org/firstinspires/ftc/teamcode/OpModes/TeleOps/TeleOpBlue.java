package org.firstinspires.ftc.teamcode.OpModes.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.OpModes.Players.Player1;
import org.firstinspires.ftc.teamcode.OpModes.Players.Player2;
import org.firstinspires.ftc.teamcode.OpModes.Robot;
import org.firstinspires.ftc.teamcode.RobotCore.RobotUtils.RobotAlliance;
import org.firstinspires.ftc.teamcode.RobotCore.RobotUtils.RobotMode;
import org.firstinspires.ftc.teamcode.RobotCore.Utils.Position;


@TeleOp(name = "BlueMeow", group = "Blue")
public class TeleOpBlue extends OpMode {
    Player1 player1 = new Player1(this);
    Player2 player2 = new Player2(this);


    /**
     *  Метод вызывается один раз при нажатии INIT
     */
    @Override
    public void init() {
        player1.init();
        player2.init();
    }

    /**
     *  Метод крутится в цикле, ожидая нажатия START
     */
    @Override
    public void init_loop() {
        telemetry.clear();
    }

    /**
     *  Метод вызывается один раз при нажатии кнопки START
     */
    @Override
    public void start() {
        player1.startTeleop();
        player2.startTeleop();
    }
    /**
     *  Метод крутится в цикле после нажатия START
     */
    @Override
    public void loop() {
//        robot.odometry.getEncPos();
//        robot.odometry.getRobotPos();
//        telemetry.addData("Gear", robot.joysticks.getGear());
//        telemetry.addData("teleGear", robot.joysticks.gearTele);
//        telemetry.addData("height", robot.teleSkope.getHeight());
//        robot.joysticks.getDpadUp();
    }

    /**
     *  Метод вызывается один раз при нажатии STOP
     */
    @Override
    public void stop() {
        player1.interrupt();
        player2.interrupt();
    }
}
