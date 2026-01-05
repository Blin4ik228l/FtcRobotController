package org.firstinspires.ftc.teamcode.Programms.Auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Modules.Examples.AutoParts.AutonomLogic;
import org.firstinspires.ftc.teamcode.Modules.Examples.AutoParts.DriveHandler;
import org.firstinspires.ftc.teamcode.Robot.RobotClass;
import org.firstinspires.ftc.teamcode.Robot.RobotParts.DriveTrain.DrivetrainParts.Odometry.Parts.MathUtils.Position2D;
import org.firstinspires.ftc.teamcode.Robot.RobotParts.TelemetrySettings;
import org.firstinspires.ftc.teamcode.TaskAndArgs.Args;

public class LinearOpModeModernized extends LinearOpMode {
    public RobotClass robot;
    private AutonomLogic autonomLogic;
    public TelemetrySettings telemetrySettings;
    public DriveHandler driveHandler;
    @Override
    public void runOpMode() throws InterruptedException {

    }
    public void run(){
        autonomLogic = new AutonomLogic(robot, this);

        while (!isStarted()){
            robot.update();

            robot.showData();
            autonomLogic.showData();

            telemetry.update();
        }

        robot.startTimer();

        while (!isStopRequested() && opModeIsActive()){
            robot.update();

            autonomLogic.execute();

            autonomLogic.showData();
            robot.showData();

            telemetry.update();
        }
    }
    public void extRun(){

    }
}
