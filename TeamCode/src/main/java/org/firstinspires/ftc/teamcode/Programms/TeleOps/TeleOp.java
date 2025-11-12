package org.firstinspires.ftc.teamcode.Programms.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Modules.Players.Pl1.Player1;
import org.firstinspires.ftc.teamcode.Modules.Players.Pl2.Player2;
import org.firstinspires.ftc.teamcode.Robot.AutomaticClass;
import org.firstinspires.ftc.teamcode.Robot.RobotClass;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "teleopBlue", group = "Blue")
public class TeleOp extends OpMode {
    Thread parallelStream, parallelStream2;
    Player2 player2;
    Player1 player1;
    RobotClass robot;
    AutomaticClass automaticClass;

    @Override
    public void init() {
        robot = new RobotClass(this, "Blue");

        player1 = new Player1(gamepad1, robot.driveTrain, this);
        automaticClass = new AutomaticClass(robot.collector , player1.joystickActivity,this);

        parallelStream = new Thread(player1);
        parallelStream.setDaemon(true);// Эта строчка мнгновено позволяет "убить" поток после завершения программы

        parallelStream2 = new Thread(automaticClass);
        parallelStream2.setDaemon(true);
    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {
        parallelStream.start();
        parallelStream2.start();
    }

    @Override
    public void loop() {
        if(automaticClass.randomizedArtifact[0] == 0) automaticClass.randomizedArtifact = player1.driveTrain.exOdometry.camera.randomizedArtifact;
        automaticClass.isVyrCompleted = player1.driveTrain.exOdometry.isVyrCompleted;
        if(automaticClass.range != player1.driveTrain.exOdometry.getRange()) automaticClass.range = player1.driveTrain.exOdometry.getRange();

        player1.showData();
        automaticClass.showData();
    }

    @Override
    public void stop() {
        parallelStream.interrupt();
        parallelStream2.interrupt();
    }

}
