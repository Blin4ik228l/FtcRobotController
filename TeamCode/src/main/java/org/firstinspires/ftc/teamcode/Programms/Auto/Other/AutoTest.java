package org.firstinspires.ftc.teamcode.Programms.Auto.Other;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Modules.Examples.AutoParts.AutonomLogic;
import org.firstinspires.ftc.teamcode.Programms.Auto.LinearOpModeModernized;
import org.firstinspires.ftc.teamcode.Robot.RobotClass;
import org.firstinspires.ftc.teamcode.Robot.RobotParts.DrivetrainParts.CameraClass;
import org.firstinspires.ftc.teamcode.Robot.TeamColor;

@Autonomous(name = "BlueLeft", group = "Blue", preselectTeleOp = "BlueMeow")
public class AutoTest extends LinearOpModeModernized {
    public RobotClass robot;
    public AutonomLogic autonomLogic;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new RobotClass(TeamColor.Color.Blue, TeamColor.StartPos.Near_wall, this);
        autonomLogic = new AutonomLogic(robot, this);

        while (!isStarted()){
            robot.update();

            robot.showData();
            autonomLogic.showData();

            telemetry.update();
        }

        robot.start();

        while (!isStopRequested() && opModeIsActive()){
            robot.update();

            autonomLogic.execute();

            autonomLogic.showData();
            robot.showData();

            telemetry.update();
        }

    }
}
