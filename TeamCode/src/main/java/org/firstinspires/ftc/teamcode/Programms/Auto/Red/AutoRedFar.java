package org.firstinspires.ftc.teamcode.Programms.Auto.Red;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Programms.Auto.LinearOpModeModernized;
import org.firstinspires.ftc.teamcode.Robot.GeneralInformation;
import org.firstinspires.ftc.teamcode.Robot.RobotClass;

@Autonomous(name = "RedFarWall", group = "Red", preselectTeleOp = "Red")
public class AutoRedFar extends LinearOpModeModernized {
    @Override
    public void runOpMode() throws InterruptedException {
        new GeneralInformation(GeneralInformation.ProgramName.Auto, GeneralInformation.Color.Red, GeneralInformation.StartPos.Far_from_wall);
        robot = new RobotClass(this);
        run();
    }
}
