package org.firstinspires.ftc.teamcode.Programms.Auto.Red;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Programms.Auto.LinearOpModeModernized;
import org.firstinspires.ftc.teamcode.Robot.GeneralInformation;
import org.firstinspires.ftc.teamcode.Robot.RobotClass;
import org.firstinspires.ftc.teamcode.Robot.RobotParts.DriveTrain.DrivetrainParts.TeamClass;

@Autonomous(name = "RedFarWall", group = "Red", preselectTeleOp = "Red")
public class AutoRedFar extends LinearOpModeModernized {
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new RobotClass(this);
        robot.setGeneralInformation(GeneralInformation.ProgramName.Auto, GeneralInformation.Color.Red, GeneralInformation.StartPos.Far_from_wall);
        run();
    }
}
