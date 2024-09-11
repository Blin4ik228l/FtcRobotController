package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

//Основная идея:
//получить полный контроль над движениями робота
@Autonomous
public class AutoNewGen extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("initing");
        waitForStart();
        speedClass();
    }

    public void speedClass(){
        while(opModeIsActive() && !isStopRequested()) {

            telemetry.update();
        }
    }
}
