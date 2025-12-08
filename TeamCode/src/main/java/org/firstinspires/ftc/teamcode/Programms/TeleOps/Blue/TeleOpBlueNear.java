package org.firstinspires.ftc.teamcode.Programms.TeleOps.Blue;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Modules.Examples.Players.InnerWardenClass;
import org.firstinspires.ftc.teamcode.Modules.Examples.Players.JoystickActivity;
import org.firstinspires.ftc.teamcode.Modules.Examples.Players.Pl1.PlayerClass1;
import org.firstinspires.ftc.teamcode.Modules.Examples.Players.Pl2.AutomaticClass;
import org.firstinspires.ftc.teamcode.Programms.TeleOps.TeleOpModernized;
import org.firstinspires.ftc.teamcode.Robot.RobotClass;
import org.firstinspires.ftc.teamcode.Robot.TeamColor;

@TeleOp(name = "TeleopBlueNearWall", group = "Blue")
public class TeleOpBlueNear extends TeleOpModernized {
    PlayerClass1 player1;

    AutomaticClass automaticClass;
    RobotClass robot;
    InnerWardenClass innerWarden;
    JoystickActivity joystickActivity;

    @Override
    public void init() {
        joystickActivity = new JoystickActivity(gamepad1, this);

        robot = new RobotClass(TeamColor.Color.Blue, TeamColor.StartPos.Near_wall, this);

        player1 = new PlayerClass1(joystickActivity, robot.driveTrain, this);
        automaticClass = new AutomaticClass(joystickActivity, robot.collector, this);

        innerWarden = new InnerWardenClass(robot, player1, automaticClass, this);

        moduleRobot = robot;

        moduleJoystickActivityPlayer1 = player1.joystickActivity;

        modulePlayer1 = player1;
        moduleInnerWarden = innerWarden;
        moduleAutomatic = automaticClass;
    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {
        robot.start();
    }

    @Override
    public void loop() {
        updateAll();

        executeAll();

        showAll();
    }

    @Override
    public void stop() {

    }
}
