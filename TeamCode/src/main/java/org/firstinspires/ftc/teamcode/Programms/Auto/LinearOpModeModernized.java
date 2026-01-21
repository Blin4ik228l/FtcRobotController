package org.firstinspires.ftc.teamcode.Programms.Auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Modules.Examples.Players.PL0.MainSystem;
import org.firstinspires.ftc.teamcode.Modules.Examples.Players.Pl1.SemiAutoPlayerClass1;
import org.firstinspires.ftc.teamcode.Modules.Examples.Players.Pl2.AutoPlayerClass2;
import org.firstinspires.ftc.teamcode.Modules.Joysticks.JoystickActivityClass;
import org.firstinspires.ftc.teamcode.Robot.RobotClass;
import org.firstinspires.ftc.teamcode.Robot.RobotParts.TelemetrySettings;

public abstract class LinearOpModeModernized extends LinearOpMode {
    public RobotClass robot;
    public TelemetrySettings telemetrySettings;
    public SemiAutoPlayerClass1 semiAutoPlayerClass1;
    public AutoPlayerClass2 autoPlayerClass2;
    public MainSystem mainSystem;
    public JoystickActivityClass joystickActivityClass;
    @Override
    public void runOpMode() throws InterruptedException {

    }
    public void run(){
        joystickActivityClass = new JoystickActivityClass(null, this);

        semiAutoPlayerClass1 = new SemiAutoPlayerClass1(joystickActivityClass, robot.drivetrain, new ElapsedTime(), this);
        autoPlayerClass2 = new AutoPlayerClass2(joystickActivityClass, robot.collector, new ElapsedTime(), this);

        mainSystem = new MainSystem(semiAutoPlayerClass1, autoPlayerClass2, this);

        telemetrySettings = new TelemetrySettings(null, this, this);

        while (!isStarted() && !isStopRequested()){
            robot.update();

            telemetrySettings.update();

            telemetrySettings.showData();
            telemetry.update();
        }

        robot.startTimer();

        while (!isStopRequested() && opModeIsActive()){
            robot.update();
            telemetrySettings.update();

            mainSystem.execute();
            semiAutoPlayerClass1.execute();
            autoPlayerClass2.execute();

            telemetrySettings.showData();
            telemetry.update();
        }
    }
    public void extRun(){

    }
}
