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

    double Gx, Gy;

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

        double rightFP = Range.clip(-gamepad1.left_stick_y - gamepad1.left_stick_x - turn, -1.0, 1.0);
        double rightBP = Range.clip(-gamepad1.left_stick_y + gamepad1.left_stick_x - turn, -1.0, 1.0);
        double leftFP =  Range.clip(gamepad1.left_stick_y - gamepad1.left_stick_x - turn, -1.0, 1.0);
        double leftBP = Range.clip(gamepad1.left_stick_y + gamepad1.left_stick_x - turn, -1.0, 1.0);

        rightF.setPower(rightFP);
        rightB.setPower(rightBP);
        leftF.setPower(leftFP);
        leftB.setPower(leftBP);

        double angle_robot = 0;
        double Rx = 0;
        double Ry = Ry = (encM.getCurrentPosition()/CONSTS.TICK_PER_CM);
        double Rad = 0;

        if(rightB.getPower() + leftF.getPower() == 0.0){
            Rx = ((encL.getCurrentPosition() + encR.getCurrentPosition())/2)/(CONSTS.TICK_PER_CM);
        }else if( rightB.getPower() != 0){
            Rx = ((encL.getCurrentPosition() - encR.getCurrentPosition())/2)/(CONSTS.TICK_PER_CM);
        }

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

        angle_robot = (Rx/(CONSTS.LENGHT_ROUND_SMALL)) * 360;
        Rad = (Rx/CONSTS.LENGHT_ROUND_SMALL);

        if(rightB.getPower() != 0){
        Gx += Rx * Math.cos(angle_robot/360) - Ry * Math.sin(angle_robot/360);
        Gy += Rx * Math.sin(angle_robot/360) + Ry * Math.cos(angle_robot/360);
        }

        telemetry.addData("Левый экодер тики", encL.getCurrentPosition());
        telemetry.addData("Правый энкодер тики", encR.getCurrentPosition());
        telemetry.addData("Серединный энкодер", encM.getCurrentPosition());

        telemetry.addData("Левый экодер см", encL.getCurrentPosition()/ (CONSTS.TICK_PER_CM));
        telemetry.addData("Правый энкодер см", encR.getCurrentPosition()/(CONSTS.TICK_PER_CM));
        telemetry.addData("Серединный энкодер см", encM.getCurrentPosition()/(CONSTS.TICK_PER_CM));

        telemetry.addData("Угол поворота робота", angle_robot );
        telemetry.addData("Радианы", Rad);


        telemetry.addData("Левый экодер тики/сек", encL.getVelocity());
        telemetry.addData("Правый энкодер тики/сек", encR.getVelocity());
        telemetry.addData("Серединный энкодер/сек", encM.getVelocity());

        telemetry.addData("Rx", Rx);
        telemetry.addData("Ry", Ry);

        telemetry.addData("Gx", Gx);
        telemetry.addData("Gy", Gy);

        telemetry.update();

        runtime.reset();

        if(gamepad1.back){
            encR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            encL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            encM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            encM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            encR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            encL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }


    }

}
