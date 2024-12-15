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
        height = ticksToCM((upStandingLeft.getCurrentPosition() + upStandingRight.getCurrentPosition())/2.0);
    }
    public synchronized void setHook(boolean openHook){
        if(openHook){
            servosService.getHook().setPosition(OPEN_POS_HOOK);
        }else {
            servosService.getHook().setPosition(CLOSE_POS_HOOK);
        }
    }

    public synchronized void setTeleskopePropAuto(double speed, double posServo, double reachableHeight){
        calculateHeight();

        double DEGREES_TO_LENGHT = posServo * 270 * reachableHeight;//Градусов до полного разложения

        double toDeadZone =  (reachableHeight - height);
        double P = DEGREES_TO_LENGHT/reachableHeight;

        double targetVel = speed * Math.signum(toDeadZone);

        setVelUpStandingTeleOp(targetVel);
        setVelHorizontalTeleOp((toDeadZone * P) / 270.0);
    }

    public synchronized void setTeleskope(double vel, double Pos){
            setVelUpStandingTeleOp(vel);
            setVelHorizontalTeleOp(Pos);
        }

    public synchronized void setTeleskopeProp(double vel, double Pos){
        calculateHeight();

        double DEAD_ZONE_HEIGHT = 121;

        double PROPRTIONAL_HEIGHT = 9;// Высота на которой телескопы будут двигаться одновременно
        double DEGREES_TO_LENGHT = OPEN_POS_HORIZONTAL * 270 * DEAD_ZONE_HEIGHT;//Градусов до полного разложения

        double toDeadZone =  (DEAD_ZONE_HEIGHT - height);
        double P = DEGREES_TO_LENGHT/DEAD_ZONE_HEIGHT;

        if((height > PROPRTIONAL_HEIGHT) ) {
            setVelUpStandingTeleOp(vel);
            setVelHorizontalTeleOp((toDeadZone * P) / 270.0);
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
        servosService.getHorizontal().setPosition(Range.clip(Pos, OPEN_POS_HORIZONTAL, CLOSE_POS_HORIZONTAL));
    }

}
