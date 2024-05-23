package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;
@TeleOp(name = "encTest", group = "huy")
public class AutoNewGen extends LinearOpMode implements CONSTS{
    DigitalChannel enc;
    OpMode op;
    ElapsedTime runtime = new ElapsedTime();
    DcMotor encL, encR, encB, lefFR, lefBC, righFR, righBC;
    double encL_ticks, encR_ticks, encB_ticks,
            encL_angle, encR_angle, encB_angle,
            x_distance, y_distance,
            angle;

    @Override
    public void runOpMode() throws InterruptedException {
        initC();
        waitForStart();

        drive_auto();

    }

    public void initC() {
        encL = hardwareMap.get(DcMotor.class, "encL");
        encR = hardwareMap.get(DcMotor.class, "encR");
        encB = hardwareMap.get(DcMotor.class, "encB");

        righBC = hardwareMap.get(DcMotor.class, "righBC");
        righFR = hardwareMap.get(DcMotor.class, "righFR");
        lefBC = hardwareMap.get(DcMotor.class, "lefBC");
        lefFR = hardwareMap.get(DcMotor.class, "lefFR");
        

    }
public void controll_motor(){
        time = runtime.seconds();
    while(!isStopRequested() && opModeIsActive()){

        double TPS_righBC = righBC.getCurrentPosition()/time;
        double TPS_righFR = righFR.getCurrentPosition()/time;
        double TPS_lefBC = lefBC.getCurrentPosition()/time;
        double TPS_lefFR = lefFR.getCurrentPosition()/time;

        double RPS_righBC = TPS_righBC/ TPR_WHEEL;
        double RPS_righFR = TPS_righFR/ TPR_WHEEL;
        double RPS_lefBC = TPS_lefBC/ TPR_WHEEL;
        double RPS_lefFR = TPS_lefFR/ TPR_WHEEL;

        runtime.reset();

        telemetry.addData("Об. в сек. правый зад:", RPS_righBC);
        telemetry.addData("Об. в сек. правый перед:", RPS_righFR);
        telemetry.addData("Об. в сек. левый зад:", RPS_lefBC);
        telemetry.addData("Об. в сек. левый перед:", RPS_lefFR);

        telemetry.addData("Тик. в сек. по номиналу у колеса:", TPS_WHEEL);

        telemetry.addData("Тик. в сек. правый зад:", TPS_righBC);
        telemetry.addData("Тик. в сек. правый перед:", TPS_righFR);
        telemetry.addData("Тик. в сек. левый зад:", TPS_lefBC);
        telemetry.addData("Тик. в сек. левый перед:", TPS_lefFR);

        telemetry.update();
    }
}
    public void drive_auto(){
        while(!isStopRequested() && opModeIsActive()){
            encL_ticks = encL.getCurrentPosition();
            encR_ticks = encR.getCurrentPosition();
            encB_ticks = encB.getCurrentPosition();

            angle = (encR_ticks - encL_ticks) / (2.0 * DIST_TO_ENC_Y_FROM_CENTER);
            y_distance = (encR_ticks + encL_ticks) / 2.0;
            x_distance = encB_ticks - angle;

            telemetry.addData("левый", encL_ticks);
            telemetry.addData("правый", encR_ticks);
            telemetry.addData("задний", encB_ticks);

            telemetry.addData("угол робота", angle);
            telemetry.addData("расттояние по боковым", y_distance);
            telemetry.addData("расстояние по заднему", x_distance);

            telemetry.update();
        }
    }
}
