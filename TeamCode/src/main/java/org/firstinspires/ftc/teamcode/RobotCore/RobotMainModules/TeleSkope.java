package org.firstinspires.ftc.teamcode.RobotCore.RobotMainModules;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

public class TeleSkope implements Module{
    public final OpMode op;
    public DcMotor upStanding;
    public DcMotor horizontal;

    public TeleSkope(OpMode op){
        this.op = op;
    }
    @Override
    public void init() {
        upStanding = op.hardwareMap.get(DcMotorEx.class, "upStanding");
        horizontal = op.hardwareMap.get(DcMotorEx.class, "horizontal");
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behabior) {
        upStanding.setZeroPowerBehavior(behabior);
        horizontal.setZeroPowerBehavior(behabior);
    }

    public void brakeMotors(){
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void setBehaviorMotor(@NonNull DcMotor motor, DcMotor.ZeroPowerBehavior behavior){
        motor.setZeroPowerBehavior(behavior);
    }

    public void floatMotors(){
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void offMotors(){
        upStanding.setPower(0);
        horizontal.setPower(0);
    }

    public void setVelUpStandingTeleOp(double Vel){
        if(Vel == 0 ){
            setBehaviorMotor(upStanding, DcMotor.ZeroPowerBehavior.BRAKE);
            return;
        }
        upStanding.setPower(Range.clip((Vel), -1.0, 1.0));
    }

    public void setVelHorizontalTeleOp(double Vel){
        if(Vel == 0 ){
            setBehaviorMotor(horizontal, DcMotor.ZeroPowerBehavior.BRAKE);
            return;
        }
        horizontal.setPower(Range.clip((Vel), -1.0, 1.0));
    }
}
