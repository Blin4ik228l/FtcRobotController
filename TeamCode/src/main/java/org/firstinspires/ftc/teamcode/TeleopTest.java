package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@TeleOp
public class TeleopTest extends OpMode {

    DcMotorEx rightB, rightF, leftB, leftF;

    DcMotorEx encM, encL, encR;

    ElapsedTime runtime = new ElapsedTime();
    @Override
    public void init() {
        rightB = hardwareMap.get(DcMotorEx.class, "rightB");
        rightF = hardwareMap.get(DcMotorEx.class, "rightF");
        leftB = hardwareMap.get(DcMotorEx.class, "leftB");
        leftF = hardwareMap.get(DcMotorEx.class, "leftF");

        encM = hardwareMap.get(DcMotorEx.class, "rightF") ;
        encL = hardwareMap.get(DcMotorEx.class, "leftF") ;
        encR =  hardwareMap.get(DcMotorEx.class, "rightB");

        rightB.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightF.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        leftF.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        leftB.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        encR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        encM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        encR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        encL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void loop() {
        runtime.milliseconds();

        double turn  =  gamepad1.right_stick_x;

        rightF.setPower(Range.clip(-gamepad1.left_stick_y - gamepad1.left_stick_x - turn, -1.0, 1.0));
        rightB.setPower(Range.clip(-gamepad1.left_stick_y + gamepad1.left_stick_x - turn, -1.0, 1.0));
        leftF.setPower(Range.clip(gamepad1.left_stick_y - gamepad1.left_stick_x - turn, -1.0, 1.0));
        leftB.setPower(Range.clip(gamepad1.left_stick_y + gamepad1.left_stick_x - turn, -1.0, 1.0));

        if(turn == 0.0 && gamepad1.left_stick_x == 0.0 && gamepad1.left_stick_y == 0.0){
            rightF.setPower(0);
            rightB.setPower(0);
            leftF.setPower(0);
            leftB.setPower(0);

            rightB.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            rightF.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            leftB.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            leftF.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        }

        telemetry.addData("Левый экодер тики", encL.getCurrentPosition());
        telemetry.addData("Правый энкодер тики", -encR.getCurrentPosition());
        telemetry.addData("Серединный энкодер", encM.getCurrentPosition());

        telemetry.addData("Левый экодер см", encL.getCurrentPosition()/ (CONSTS.TICK_PER_CM));
        telemetry.addData("Правый энкодер см", -encR.getCurrentPosition()/(CONSTS.TICK_PER_CM));
        telemetry.addData("Серединный энкодер см", encM.getCurrentPosition()/(CONSTS.TICK_PER_CM));

if (rightB.getPower() + leftF.getPower() == 0){
    telemetry.addData("Угол поворота робота", (((encL.getCurrentPosition()-encR.getCurrentPosition()/(CONSTS.TICK_PER_CM))/2)
        /(CONSTS.LENGHT_ROUND_SMALL)) * 360);}
else{
    telemetry.addData("Угол поворота робота", (((encL.getCurrentPosition()+encR.getCurrentPosition()/(CONSTS.TICK_PER_CM))/2)
            /(CONSTS.LENGHT_ROUND_SMALL)) * 360);
}

        telemetry.addData("Левый экодер тики/сек", encL.getVelocity());
        telemetry.addData("Правый энкодер тики/сек", -encR.getVelocity());
        telemetry.addData("Серединный энкодер/сек", encM.getVelocity());
        telemetry.update();

        runtime.reset();


    }

}
