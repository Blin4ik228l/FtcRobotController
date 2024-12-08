package org.firstinspires.ftc.teamcode.RobotCore.RobotMainModules;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.RobotCore.Utils.CONSTS;

public class TeleSkope implements Module{
    public final OpMode op;
    private DcMotor upStandingLeft;
    private DcMotor upStandingRight;
    private Servo horizontal;
    private double height;

    public TeleSkope(OpMode op){
        this.op = op;
    }
    @Override
    public void init() {
        upStandingLeft = op.hardwareMap.get(DcMotorEx.class, "upStanding");
        upStandingRight = op.hardwareMap.get(DcMotorEx.class, "horizontal");
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        upStandingLeft.setZeroPowerBehavior(behavior);
        upStandingRight.setZeroPowerBehavior(behavior);
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
        upStandingLeft.setPower(0);
        upStandingRight.setPower(0);
    }

    public double getHeight(){
        return height;
    }

    public void calculateHeight(){
        height = ((upStandingLeft.getCurrentPosition() + upStandingRight.getCurrentPosition())/2.0)/CONSTS.CM_PER_TICK_BARABAN;
    }

    public void setTeleskope(double vel, double Pos){
        double DEAD_ZONE_HEIGHT = 123;
        double DEAD_ZONE_LENGHT = 75;
        double PROPRTIONAL_HEIGHT = 9;// Высота на которой телескопы будут двигаться одновременно
        double DEGREES_TO_LENGHT = 90;//Градусов до полного разложения

        double toDeadZone = DEAD_ZONE_HEIGHT - (DEAD_ZONE_HEIGHT - height);
        double P = DEGREES_TO_LENGHT/DEAD_ZONE_HEIGHT;

//        if(vel == 0 && Pos == 0){
//            return;
//        }

        double oldHEIGHT = height;
        calculateHeight();

        double deltaHEIGHT = oldHEIGHT - height;

        if(height > PROPRTIONAL_HEIGHT){
            setVelUpStandingTeleOp(vel);
            setVelHorizontalTeleOp((toDeadZone * P)/360.0);
        }else{
            setVelUpStandingTeleOp(vel);
            setVelHorizontalTeleOp(Pos);
        }

    }

    public void setVelUpStandingTeleOp(double Vel){
        if(Vel == 0 ){
            offMotors();
            return;
        }
        upStandingLeft.setPower(Range.clip((Vel), -1.0, 1.0));
        upStandingRight.setPower(Range.clip((-Vel), -1.0, 1.0));
    }

    public void setVelHorizontalTeleOp(double Pos){
        if(Pos == 0 ){
            return;
        }
        horizontal.setPosition(Pos);
    }

}
