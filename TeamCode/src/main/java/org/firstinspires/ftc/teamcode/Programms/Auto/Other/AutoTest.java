package org.firstinspires.ftc.teamcode.Programms.Auto.Other;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Modules.Examples.AutoParts.DriveHandler;
import org.firstinspires.ftc.teamcode.Programms.Auto.LinearOpModeModernized;
import org.firstinspires.ftc.teamcode.Robot.RobotClass;
import org.firstinspires.ftc.teamcode.Robot.RobotParts.DriveTrain.DrivetrainParts.Odometry.Parts.MathUtils.Position2D;
import org.firstinspires.ftc.teamcode.Robot.RobotParts.DriveTrain.DrivetrainParts.TeamClass;
import org.firstinspires.ftc.teamcode.TaskAndArgs.Args;

@Autonomous(name = "Test", group = "Unknown")
public class AutoTest extends LinearOpModeModernized {

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new RobotClass(TeamClass.Color.Blue, TeamClass.StartPos.Near_wall, this);

        extRun();
    }

    @Override
    public void extRun() {
        driveHandler = new DriveHandler(robot.driveTrain, this);
        driveHandler.setArgs(new Args.DriveArgs(
                new Position2D(100, 100 , Math.toRadians(50)), 200
        ));

        while (!driveHandler.isDone){
            driveHandler.execute();
            driveHandler.showData();
        }
    }
}
