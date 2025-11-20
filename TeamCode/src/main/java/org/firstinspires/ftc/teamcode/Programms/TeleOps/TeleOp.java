package org.firstinspires.ftc.teamcode.Programms.TeleOps;

import org.firstinspires.ftc.teamcode.Modules.Players.InnerWardenClass;
import org.firstinspires.ftc.teamcode.Modules.Players.JoystickActivity;
import org.firstinspires.ftc.teamcode.Modules.Players.Pl1.PlayerClass1;
import org.firstinspires.ftc.teamcode.Modules.Players.Pl2.AutomaticClass;
import org.firstinspires.ftc.teamcode.Robot.RobotClass;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "teleopBlue", group = "Blue")
public class TeleOp extends TeleOpModernized {
    PlayerClass1 player1;

    AutomaticClass automaticClass;
    RobotClass robot;
    InnerWardenClass innerWarden;
    JoystickActivity joystickActivity;

    @Override
    public void init() {
        joystickActivity = new JoystickActivity(gamepad1, this);

        robot = new RobotClass(this, "Red");

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
