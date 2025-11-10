package org.firstinspires.ftc.teamcode.Programms.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Modules.Players.Pl1.Player1;
import org.firstinspires.ftc.teamcode.Modules.Players.Pl2.Player2;
import org.firstinspires.ftc.teamcode.Robot.AutomaticClass;
import org.firstinspires.ftc.teamcode.Robot.RobotClass;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "teleopBlue", group = "Blue")
public class TeleOp extends OpMode {
    Thread parallelStream;
    Player2 player2;
    Player1 player1;
    RobotClass robot;

    AutomaticClass automaticClass;

    @Override
    public void init() {
        robot = new RobotClass(this, "Blue");
        player1 = new Player1(gamepad1, robot.driveTrain, this);
        player2 = new Player2(gamepad2, robot.collector, this);

        automaticClass = new AutomaticClass(player1, player2,this);
//
//        parallelStream = new Thread(player2);
//        parallelStream.setDaemon(true);// Эта строчка позволяет "убить" поток после завершения программы
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
        player1.play();
        player2.play();
        automaticClass.execute();
        automaticClass.showData();
    }

    @Override
    public void stop() {

    }

}
