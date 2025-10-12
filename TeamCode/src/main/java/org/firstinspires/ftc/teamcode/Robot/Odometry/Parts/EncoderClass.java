package org.firstinspires.ftc.teamcode.Robot.Odometry.Parts;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class EncoderClass {
    public EncoderClass(@NonNull OpMode op){
        encLeft = op.hardwareMap.get(DcMotorEx.class, "leftB");
        encRight = op.hardwareMap.get(DcMotorEx.class, "rightB");
        encMid = op.hardwareMap.get(DcMotorEx.class, "rightF" );

        encRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);                  // обновляем правый энкодер
        encLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);                  // обновляем левый энкодер
        encMid.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);                  // обновляем средний энкодер

        encMid.setMode(DcMotor.RunMode.RUN_USING_ENCODER);                     // запускаем средний энкодер
        encRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);                     // запускаем правый энкодер
        encLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public double COUNTS_PER_ENCODER_REV = 2000;
    public double DRIVE_GEAR_REDUCTION = 1;
    public double ENC_WHEEL_DIAM_CM = 4.8;
    public double COUNTS_PER_CM = (COUNTS_PER_ENCODER_REV * DRIVE_GEAR_REDUCTION)/
            (ENC_WHEEL_DIAM_CM * Math.PI);

    private final DcMotorEx encLeft;
    private final DcMotorEx encMid;
    private final DcMotorEx encRight;

    public double getCurrentPosLeft(){
        return -encLeft.getCurrentPosition();
    }

    public double getCurrentPosMid(){
        return -encMid.getCurrentPosition();
    }

    public double getCurrentPosRight(){
        return -encRight.getCurrentPosition();
    }

    public double getCurrentVelocityLeft(){
        return -encLeft.getVelocity();
    }

    public double getCurrentVelocityMid(){
        return -encMid.getVelocity();
    }

    public double getCurrentVelocityRight(){
        return -encRight.getVelocity();
    }

}
