package org.firstinspires.ftc.teamcode.Programms.TeleOps.Red;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Modules.Examples.Players.JoystickActivity;
import org.firstinspires.ftc.teamcode.Modules.Examples.Players.Pl1.PlayerClass1;
import org.firstinspires.ftc.teamcode.Modules.Examples.Players.Pl2.PlayerClass2;
import org.firstinspires.ftc.teamcode.Robot.RobotClass;
import org.firstinspires.ftc.teamcode.Robot.TeamColor;

@TeleOp(name = "TeleopRedNearWall", group = "Red")
public class TeleOpRedNear extends OpMode {
    Thread parallelStream;
    PlayerClass2 dimas;
    PlayerClass1 leva;
    RobotClass robot;
    JoystickActivity joystickActivity;

    @Override
    public void init() {
        joystickActivity = new JoystickActivity(gamepad1, this);
        robot = new RobotClass(TeamColor.Color.Red, TeamColor.StartPos.Near_wall, this);

        leva = new PlayerClass1(joystickActivity, robot.driveTrain, this);
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
