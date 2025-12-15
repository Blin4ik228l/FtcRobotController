package org.firstinspires.ftc.teamcode.Programms.Auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Modules.Examples.AutoParts.AutonomLogic;
import org.firstinspires.ftc.teamcode.Modules.Examples.Players.InnerWardenClass;
import org.firstinspires.ftc.teamcode.Modules.Examples.Players.JoystickActivityClass;
import org.firstinspires.ftc.teamcode.Modules.Examples.Players.Pl1.PlayerClass1;
import org.firstinspires.ftc.teamcode.Modules.Examples.Players.Pl2.AutoPlayerClass;
import org.firstinspires.ftc.teamcode.Modules.Types.ExecutableModule;
import org.firstinspires.ftc.teamcode.Modules.Types.UpdatableModule;
import org.firstinspires.ftc.teamcode.Robot.RobotClass;
import org.firstinspires.ftc.teamcode.Robot.RobotParts.DrivetrainParts.TeamColorClass;

public class LinearOpModeModernized extends LinearOpMode {
    public RobotClass robot;
    private AutonomLogic autonomLogic;
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
