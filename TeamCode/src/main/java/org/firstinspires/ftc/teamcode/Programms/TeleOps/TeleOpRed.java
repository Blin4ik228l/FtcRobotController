package org.firstinspires.ftc.teamcode.Programms.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Modules.Players.Pl1.PlayerClass1;
import org.firstinspires.ftc.teamcode.Modules.Players.Pl2.PlayerClass2;
import org.firstinspires.ftc.teamcode.Robot.RobotClass;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "teleopRed", group = "Red")
public class TeleOpRed extends OpMode {
    Thread parallelStream;
    PlayerClass2 dimas;
    PlayerClass1 leva;
    RobotClass robot;

    @Override
    public void init() {
        robot = new RobotClass(this, "Red");
        leva = new PlayerClass1(gamepad1, robot.driveTrain, this);
//        dimas = new Player2(gamepad2, robot.teleSkope, this);

    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {
//        parallelStream.start();
    }

    @Override
    public void loop() {

    }

    @Override
    public void stop() {

    }

}
