package org.firstinspires.ftc.teamcode.Programms.Auto.Other;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Programms.Auto.LinearOpModeModernized;
import org.firstinspires.ftc.teamcode.Robot.GeneralInformation;
import org.firstinspires.ftc.teamcode.Robot.RobotClass;
import org.firstinspires.ftc.teamcode.Robot.RobotParts.DriveTrain.DrivetrainParts.Odometry.Parts.MathUtils.Position2D;
import org.firstinspires.ftc.teamcode.Robot.RobotParts.TelemetrySettings;
import org.firstinspires.ftc.teamcode.TaskAndArgs.Args;

@Autonomous(name = "TestAuto", group = "Unknown")
public class AutoTest extends LinearOpModeModernized {

    @Override
    public void runOpMode() throws InterruptedException {
        new GeneralInformation(GeneralInformation.ProgramName.Auto, GeneralInformation.Color.Blue, GeneralInformation.StartPos.Near_wall);
        robot = new RobotClass(this);
        telemetrySettings = new TelemetrySettings(null, this, this);

        extRun();
    }

    @Override
    public void extRun() {

        while (!isStarted()){
            robot.update();
            telemetrySettings.update();

            telemetrySettings.showData();

            telemetry.update();
        }

        while (!isStopRequested() && opModeIsActive()){
            robot.update();
            telemetrySettings.update();

            telemetrySettings.showData();

            telemetry.update();
        }
    }
}
