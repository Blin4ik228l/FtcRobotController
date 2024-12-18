package org.firstinspires.ftc.teamcode.RobotCore.RobotMainModules.Physical;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.RobotCore.RobotMainModules.Module;
import org.firstinspires.ftc.teamcode.RobotCore.Utils.CONSTS;
import org.firstinspires.ftc.teamcode.RobotCore.Utils.CONSTSTELESKOPE;

public class TeleSkope implements Module, CONSTSTELESKOPE {
    public final OpMode op;
    public final ServosService servosService;

    private DcMotor upStandingLeft;
    private DcMotor upStandingRight;
    private double height;

    public TeleSkope(OpMode op, ServosService servosService){
        this.op = op;
        this.servosService = servosService;
    }
    @Override
    public void init() {
        upStandingLeft = op.hardwareMap.get(DcMotorEx.class, "upStandingLeft");
        upStandingRight = op.hardwareMap.get(DcMotorEx.class, "upStandingRight");

        upStandingLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        upStandingRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        upStandingLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        upStandingRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        brakeMotors();
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

    public DcMotor getUpStandingLeft() {
        return upStandingLeft;
    }

    public DcMotor getUpStandingRight() {
        return upStandingRight;
    }

    public double getHeight(){
        return height;
    }

    public double ticksToCM(double ticks){
        return ticks / CONSTS.TICK_PER_CM_BARABAN;
    }

    public double cmToTicks(double cm){
        return cm * CONSTS.TICK_PER_CM_BARABAN;
    }

    public void calculateHeight(){
        if(upStandingLeft.getCurrentPosition() < upStandingRight.getCurrentPosition()){
            height = ticksToCM((upStandingLeft.getCurrentPosition()));
        } else if (upStandingRight.getCurrentPosition() < upStandingLeft.getCurrentPosition()) {
            height = ticksToCM((upStandingLeft.getCurrentPosition()));
        }else {
            height = ticksToCM((upStandingLeft.getCurrentPosition() + upStandingRight.getCurrentPosition()) / 2.0);
        }
    }
    public synchronized void setHook(double Pos){
        servosService.getHook().setPosition(Range.clip(Pos, CLOSE_POS_HOOK, OPEN_POS_HOOK));
    }

    public synchronized void setTeleskopePropAuto(double speed, double posServo, double reachableHeight){
        calculateHeight();

        double P = (CLOSE_POS_HORIZONTAL - posServo)/(reachableHeight);

        double propLen = CLOSE_POS_HORIZONTAL - (height) * P;

        double targetVel = speed * Math.signum(reachableHeight - height);

        setVelUpStandingTeleOp(targetVel);
        setVelHorizontalTeleOp(propLen);
    }

    public synchronized void setTeleskope(double vel, double Pos){
            setVelUpStandingTeleOp(vel);
            setVelHorizontalTeleOp(Pos);
        }

    public synchronized void setTeleskopeProp(double vel, double Pos){
        calculateHeight();

        double DEAD_ZONE_HEIGHT = 126;

        double PROPRTIONAL_HEIGHT = 9;// Высота на которой телескопы будут двигаться одновременно

        double P = (CLOSE_POS_HORIZONTAL - OPEN_POS_HORIZONTAL)/(DEAD_ZONE_HEIGHT - PROPRTIONAL_HEIGHT);

        double propLen = CLOSE_POS_HORIZONTAL - (height - PROPRTIONAL_HEIGHT) * P;

        if((height > PROPRTIONAL_HEIGHT) ) {
            setVelUpStandingTeleOp(vel);
            setVelHorizontalTeleOp(propLen);
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
        if(Math.abs(upStandingRight.getCurrentPosition() - upStandingLeft.getCurrentPosition()) > 2 && upStandingRight.getCurrentPosition() < upStandingLeft.getCurrentPosition() && Vel > 0){
            upStandingLeft.setPower(0);
            upStandingRight.setPower(Range.clip((-Vel), -1.0, 1.0));
        } else if (Math.abs(upStandingRight.getCurrentPosition() - upStandingLeft.getCurrentPosition()) > 2 && upStandingRight.getCurrentPosition() < upStandingLeft.getCurrentPosition() && Vel < 0) {
            upStandingLeft.setPower(Range.clip((Vel), -1.0, 1.0));
            upStandingRight.setPower(0);
        } else if(Math.abs(upStandingRight.getCurrentPosition() - upStandingLeft.getCurrentPosition()) > 2 && upStandingLeft.getCurrentPosition() < upStandingRight.getCurrentPosition() && Vel > 0){
            upStandingLeft.setPower(Range.clip((Vel), -1.0, 1.0));
            upStandingRight.setPower(0);
        }else if(Math.abs(upStandingRight.getCurrentPosition() - upStandingLeft.getCurrentPosition()) > 2 && upStandingLeft.getCurrentPosition() < upStandingRight.getCurrentPosition() && Vel < 0){
            upStandingLeft.setPower(0);
            upStandingRight.setPower(Range.clip((-Vel), -1.0, 1.0));
        }else {
            upStandingLeft.setPower(Range.clip((Vel), -1.0, 1.0));
            upStandingRight.setPower(Range.clip((-Vel), -1.0, 1.0));
        }

    }

    public void setVelHorizontalTeleOp(double Pos){
        servosService.getHorizontal().setPosition(Range.clip(Pos, OPEN_POS_HORIZONTAL, CLOSE_POS_HORIZONTAL));
    }

}
