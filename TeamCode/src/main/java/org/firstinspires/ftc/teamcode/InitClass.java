package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.openftc.easyopencv.PipelineRecordingParameters;

import java.util.Base64;

public class InitClass extends AutoNewGen {
    //Моторы на телеге
    DcMotorEx rightB, rightF, leftF, leftB;

    //Моторы не на телеге
    DcMotorEx sampleMotor;

    //Внешние энкодеры
    DcMotorEx encB, encL, encR;

    //Серваки
    Servo sHook;

    NormalizedColorSensor colorSensor;

    public void initDevices(){
//        rightB = hardwareMap.get(DcMotorEx.class, "rightB");//<----Зелённым пишется так, как в телефоне
//        rightF = hardwareMap.get(DcMotorEx.class, "rightF");
//        leftB = hardwareMap.get(DcMotorEx.class, "leftB");
//        leftF = hardwareMap.get(DcMotorEx.class, "leftF");



        sHook = hardwareMap.get(Servo.class, "sHook");
//        encB = hardwareMap.get(DcMotorEx.class, "encB");
//        encL = hardwareMap.get(DcMotorEx.class, "encL");
//        encR = hardwareMap.get(DcMotorEx.class, "encR");


    }
    public void resetAllEncoders(){
        rightB.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightF.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftF.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftB.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        encR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        encB.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        encL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void brakeAllMotors(){
        rightB.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightF.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftB.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftF.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }
}
