package org.firstinspires.ftc.teamcode.Game.Programms.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Game.Players.MultiPlayer.Driver1.Driver1;
import org.firstinspires.ftc.teamcode.Game.Players.MultiPlayer.Driver2.Driver2;
import org.firstinspires.ftc.teamcode.Game.Programms.Robot;
import org.firstinspires.ftc.teamcode.Game.Robot.Logic.RobotUtils.RobotAlliance;
import org.firstinspires.ftc.teamcode.Game.Robot.Logic.RobotUtils.RobotMode;
import org.firstinspires.ftc.teamcode.Game.Robot.Logic.Utils.Position;

@TeleOp(name = "RedMeow", group = "Red")
public class TeleOpRed extends OpMode {
    Robot robot;

    Driver1 name1;
    Driver2 name2;
    /**
     *  Метод вызывается один раз при нажатии INIT
     */
    @Override
    public void init() {
        name1 = new Driver1(this);
        name2 = new Driver2(this);

        robot = new Robot(RobotMode.TELEOP, RobotAlliance.RED, this, new Position());

        robot.init();

        robot.op.telemetry.addLine("Init End");
    }

    /**
     *  Метод крутится в цикле, ожидая нажатия START
     */
    @Override
    public void init_loop() {
        robot.op.telemetry.clear();
    }

    /**
     *  Метод вызывается один раз при нажатии кнопки START
     */
    @Override
    public void start() {}

    /**
     *  Метод крутится в цикле после нажатия START
     */
    @Override
    public void loop() {
        name1.functional();
        name2.functional();
    }

    /**
     *  Метод вызывается один раз при нажатии STOP
     */
    @Override
    public void stop() {
        name1.interruptJoystick();
        name2.interruptJoystick();
    }
}
