package org.firstinspires.ftc.teamcode.Programms.Auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Modules.Examples.Players.Pl1.SemiAutoPlayerClass1;
import org.firstinspires.ftc.teamcode.Modules.Examples.Players.Pl2.AutoPlayerClass2;
import org.firstinspires.ftc.teamcode.Modules.Joysticks.JoystickActivityClass;
import org.firstinspires.ftc.teamcode.Robot.RobotClass;
import org.firstinspires.ftc.teamcode.Robot.RobotParts.TelemetrySettings;

public abstract class LinearOpModeModernized extends LinearOpMode {
    public RobotClass robot;
    public TelemetrySettings telemetrySettings;
    public AutoPlayerClass2 autoPlayerClass2;
    public SemiAutoPlayerClass1 semiAutoPlayerClass1;
    public JoystickActivityClass joystickActivityClass;
    public int iterationCount = 1;
    public ElapsedTime updateTime;
    @Override
    public void runOpMode() throws InterruptedException {

    }
    public void run(){
        joystickActivityClass = new JoystickActivityClass(null, this);
        telemetrySettings = new TelemetrySettings(null, this, this);
        autoPlayerClass2 = new AutoPlayerClass2(joystickActivityClass, robot.collector, new ElapsedTime(), this);
        semiAutoPlayerClass1 = new SemiAutoPlayerClass1(joystickActivityClass, robot.drivetrain, new ElapsedTime(), this);
        updateTime = new ElapsedTime();

        while (!isStarted()){
            robot.update();

            robot.showData();

            autoPlayerClass2.showData();
            telemetry.update();
        }

        robot.startTimer();

        while (!isStopRequested() && opModeIsActive()){
            robot.setIteration(iterationCount);

            robot.update();

            autoPlayerClass2.setFields(robot.drivetrain.positionRobotController);

            semiAutoPlayerClass1.execute();
//            autoPlayerClass.execute();

            semiAutoPlayerClass1.showData();
            autoPlayerClass2.showData();
            robot.showData();

            iterationCount++;
            updateTime.reset();
            telemetry.update();
        }
    }
    public void extRun(){

    }
}
