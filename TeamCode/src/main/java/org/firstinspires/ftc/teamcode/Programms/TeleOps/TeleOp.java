package org.firstinspires.ftc.teamcode.Programms.TeleOps;

import org.firstinspires.ftc.teamcode.Modules.InnerWardenClass;
import org.firstinspires.ftc.teamcode.Modules.Players.Pl1.PlayerClass1;
import org.firstinspires.ftc.teamcode.Modules.Players.Pl2.AutomaticClass;
import org.firstinspires.ftc.teamcode.Robot.RobotClass;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "teleopBlue", group = "Blue")
public class TeleOp extends TeleOpModernized {
    PlayerClass1 player1;
    AutomaticClass automaticClass;
    RobotClass robot;

    InnerWardenClass innerWarden;

    @Override
    public void init() {
        robot = new RobotClass(this, "Blue");

        player1 = new PlayerClass1(gamepad1, robot.driveTrain, this);
        automaticClass = new AutomaticClass(gamepad1, robot.collector, this);

        innerWarden = new InnerWardenClass(robot, player1, automaticClass, this);

        moduleJoystickActivity = new ExecuteModule(player1.joystickActivity);

        moduleCamera = new ExecuteModule(robot.cameraClass);

        modulePlayer1 = new ExecuteModule(player1);

        moduleAutomaticClass = new ExecuteModule(automaticClass);
        moduleMotorsController = new ExecuteModule(automaticClass.motorsController);
        moduleColorSensor = new ExecuteModule(automaticClass.collector.colorSensor);

        moduleInnerWarden = new ExecuteModule(innerWarden);
    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {
        startExecute();
    }

    @Override
    public void loop() {
        player1.showData();
        automaticClass.showData();
    }

    @Override
    public void stop() {
        interruptAll();
    }
}
