package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

import java.text.Format;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TelePOP", group="Lagrange")

public class TeleOp extends OpMode {

    DcMotor leftFront;
    DcMotor leftBack;
    DcMotor rightFront;
    DcMotor rightBack;

    Servo servo1;




    @Override
    public void init() {
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        servo1 = hardwareMap.get(Servo.class, "servo");

        servo1.setPosition(0.0);

        leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


    }

    public void init_loop() {
        telemetry.addData("Позиция", leftBack.getCurrentPosition());

    };

    public void start() {


    };

    @Override
    public void loop() {
        double y = -gamepad1.left_stick_y;
        double polozhenyeX = gamepad1.left_stick_x;
        double angle = gamepad1.right_stick_x;

        if(gamepad1.a){

        }
        else{

        }

        int countB = 0;

        switch (countB){
            case 1:
                leftFront.setPower(1);
                break;
            case 2:
        }


        setPowerMotors(polozhenyeX, y, angle);
    }

    public void stop() {


    };

    public void setPowerMotors(double x, double y, double angle){
        leftFront.setPower(x + y + angle);
        leftBack.setPower(-x + y + angle);
        rightBack.setPower(x + y - angle);
        rightFront.setPower(-x + y - angle);
    }
}
