package org.firstinspires.ftc.teamcode.Programms.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Modules.Players.Pl1.Player1;
import org.firstinspires.ftc.teamcode.Modules.Players.Pl2.Player2;
import org.firstinspires.ftc.teamcode.Modules.RobotClass;

public class TeleOp extends OpMode {
    Thread parallelStream;
    Player2 dimas;
    Player1 leva;
    RobotClass robot;

    @Override
    public void init() {
        robot = new RobotClass(this);
        leva = new Player1(gamepad1, robot.driveTrain, this);
        dimas = new Player2(gamepad2, robot.teleSkope, this);

        parallelStream = new Thread(dimas);
        parallelStream.setDaemon(true);// Эта строчка позволяет "убить" поток после завершения программы
    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {
        parallelStream.start();
    }

    @Override
    public void loop() {
        leva.play();
    }

    @Override
    public void stop() {

    }
}
