package org.firstinspires.ftc.teamcode.OpModes.Autonoms;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotCore.RobotSubsystems.RobotCore;
import org.firstinspires.ftc.teamcode.RobotCore.TaskUtils.RobotAlliance;
import org.firstinspires.ftc.teamcode.RobotCore.TaskUtils.RobotMode;

@Autonomous(name = "RedLeft", group = "Red", preselectTeleOp = "TeleOpRed")
public class AutoRedLeft extends LinearOpMode {

    RobotCore my_robot = new RobotCore(RobotMode.AUTO, RobotAlliance.RED);

    @Override
    public void runOpMode() throws InterruptedException {

    }
}
