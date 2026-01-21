package org.firstinspires.ftc.teamcode.Robot.RobotParts.DriveTrain.DrivetrainParts.Odometry.Parts;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Modules.Types.Module;

public class EncoderClass extends Module {
    public EncoderClass(OpMode op){
        super(op);

        encLeft = hardwareMap.get(DcMotorEx.class, "leftF");
        encMid = hardwareMap.get(DcMotorEx.class, "rightB" );
        encRight = hardwareMap.get(DcMotorEx.class, "rightF");

        encRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);                  // обновляем правый энкодер
        encLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);                  // обновляем левый энкодер
        encMid.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);                  // обновляем средний энкодер

        encLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        encMid.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // запускаем средний энкодер
        encRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // запускаем правый энкодер

        telemetry.addLine("Encoders Inited");
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

    @Override
    public void showData() {
        telemetry.addLine("===ENCODERS===");
        telemetry.addData("Left", getCurrentPosLeft());
        telemetry.addData("Right", getCurrentPosRight());
        telemetry.addData("Mid", getCurrentPosMid());
        telemetry.addLine();
    }
}
