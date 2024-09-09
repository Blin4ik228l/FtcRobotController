package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

public class InitClass extends AutoNewGen {
    //Моторы на телеге
    DcMotorEx rightB, rightF, leftF, leftB;

    //Моторы не на телеге
    DcMotor sampleMotor;

    //Внешние энкодеры
    DcMotor encB, encL, encR;

    //Серваки
    Servo sampleServo;

    public void initDevices(){
        rightB = hardwareMap.get(DcMotorEx.class, "rightB");//<----Зелённым пишется так, как в телефоне
        rightF = hardwareMap.get(DcMotorEx.class, "rightF");
        leftB = hardwareMap.get(DcMotorEx.class, "leftB");
        leftF = hardwareMap.get(DcMotorEx.class, "leftF");

        encB = hardwareMap.get(DcMotor.class, "encB");
        encL = hardwareMap.get(DcMotor.class, "encL");
        encR = hardwareMap.get(DcMotor.class, "encR");
    }
    public void resetAllEncoders(){
        rightB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        encR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void brakeAllMotors(){
        rightB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
}
