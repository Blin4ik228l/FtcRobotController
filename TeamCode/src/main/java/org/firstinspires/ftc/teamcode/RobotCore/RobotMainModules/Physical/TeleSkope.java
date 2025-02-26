package org.firstinspires.ftc.teamcode.RobotCore.RobotMainModules.Physical;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Consts.CONSTS;
import org.firstinspires.ftc.teamcode.Consts.CONSTSTELESKOPE;
import org.firstinspires.ftc.teamcode.RobotCore.RobotMainModules.Module;
import org.firstinspires.ftc.teamcode.RobotCore.RobotStatus.EncoderStatus;
import org.firstinspires.ftc.teamcode.RobotCore.RobotStatus.MotorsStatus;
import org.firstinspires.ftc.teamcode.RobotCore.RobotStatus.TeleskopeStatus;

public class TeleSkope implements Module, CONSTSTELESKOPE {
    public final OpMode op;
    public final ServosService servosService;

    private DcMotor upStandingLeft;
    private DcMotor upStandingRight;
    private double height;

    public EncoderStatus leftEncUpSt;
    public EncoderStatus rightEncUpSt;

    private double leftEncUpold;
    private double rightEncUpold;

    public TeleskopeStatus motorsTeleskopeSt;

    public TeleSkope(OpMode op, ServosService servosService){
        this.op = op;
        this.servosService = servosService;
    }
    @Override
    public void init() {
        leftEncUpSt = EncoderStatus.ZeroDelta;
        rightEncUpSt = EncoderStatus.ZeroDelta;
        motorsTeleskopeSt = TeleskopeStatus.Normal;

        upStandingLeft = op.hardwareMap.get(DcMotorEx.class, "upStandingLeft");
        upStandingRight = op.hardwareMap.get(DcMotorEx.class, "upStandingRight");

        upStandingLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        upStandingRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        upStandingLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        upStandingRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        brakeMotors();

        op.telemetry.addLine("Teleskope Inited");
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

    public MotorsStatus offMotors(){
        upStandingLeft.setPower(0);
        upStandingRight.setPower(0);

        return MotorsStatus.Stopped;
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
        double leftEncUp = ticksToCM(upStandingLeft.getCurrentPosition());
        double deltaLeftEncUp = leftEncUp - leftEncUpold;
        leftEncUpold = leftEncUp;

        double rightEncUp = ticksToCM(upStandingRight.getCurrentPosition());
        double deltaRightEncUp = rightEncUp - rightEncUpold;
        rightEncUpold = rightEncUp;

        if (deltaRightEncUp == 0 && deltaLeftEncUp == 0) {
            leftEncUpSt = EncoderStatus.ZeroDelta;
            rightEncUpSt = EncoderStatus.ZeroDelta;
            return;
        }

        if(deltaRightEncUp == 0) rightEncUpSt = EncoderStatus.ZeroDelta;
        else if (deltaRightEncUp < 0.05) rightEncUpSt = EncoderStatus.SmallDelta;
        else rightEncUpSt = EncoderStatus.InMoving;

        if(deltaLeftEncUp == 0) leftEncUpSt = EncoderStatus.ZeroDelta;
        else if (deltaLeftEncUp < 0.05) leftEncUpSt = EncoderStatus.SmallDelta;
        else leftEncUpSt = EncoderStatus.InMoving;

        height += (deltaRightEncUp + deltaLeftEncUp) / 2.0;
    }

    public synchronized void setHook(double Pos){
        servosService.getHook().setPosition(Pos);
    }

    public synchronized void setFlip(double Pos){
        servosService.getFlip().setPosition(Pos);
    }

    public synchronized void setTeleskopePropAuto(double speed, double posServo, double reachableHeight){
        calculateHeight();

        double P = (CLOSE_POS_HORIZONTAL - posServo)/(reachableHeight);

        double propLen = CLOSE_POS_HORIZONTAL - (height) * P;

        double targetVel = speed * Math.signum(reachableHeight - height);

        setVelUpStandingTeleOp(targetVel);
        setPosHorizontalTeleOp(propLen);

    }

    public synchronized MotorsStatus setTeleskopeAuto(double speed, double targetHeight){
        calculateHeight();

        double targetVel = speed * Math.signum(targetHeight - height);

        if(motorsTeleskopeSt == TeleskopeStatus.Normal) setVelUpStandingTeleOp(targetVel);
        else offMotors();

        return speed == 0 ? MotorsStatus.Stopped : MotorsStatus.Powered;
    }

    public synchronized void setTeleskope(double vel, double Pos){
            setVelUpStandingTeleOp(vel);
            setPosHorizontalTeleOp(Pos);
        }

    public synchronized void setTeleskopeProp(double vel, double Pos){
        calculateHeight();

        double DEAD_ZONE_HEIGHT = 106;

        double PROPRTIONAL_HEIGHT = 15;// Высота на которой телескопы будут двигаться одновременно

        double P = (CLOSE_POS_HORIZONTAL - 0.2)/(DEAD_ZONE_HEIGHT - PROPRTIONAL_HEIGHT);

        double propLen = CLOSE_POS_HORIZONTAL - (height - PROPRTIONAL_HEIGHT) * P;

        if((height > PROPRTIONAL_HEIGHT) ) {
            setVelUpStandingTeleOp(vel);
            setPosHorizontalTeleOp(propLen);
        }else{
            setVelUpStandingTeleOp(vel);
            setPosHorizontalTeleOp(Pos);
        }

    }

    public synchronized void setVelUpStandingTeleOp(double Vel){
        if(Vel == 0 ){
            offMotors();
            return;
        }
//        if(Math.abs(upStandingRight.getCurrentPosition() - upStandingLeft.getCurrentPosition()) > 2 && upStandingRight.getCurrentPosition() < upStandingLeft.getCurrentPosition() && Vel > 0){
//            upStandingLeft.setPower(0);
//            upStandingRight.setPower(Range.clip((-Vel), -1.0, 1.0));
//        } else if (Math.abs(upStandingRight.getCurrentPosition() - upStandingLeft.getCurrentPosition()) > 2 && upStandingRight.getCurrentPosition() < upStandingLeft.getCurrentPosition() && Vel < 0) {
//            upStandingLeft.setPower(Range.clip((Vel), -1.0, 1.0));
//            upStandingRight.setPower(0);
//        } else if(Math.abs(upStandingRight.getCurrentPosition() - upStandingLeft.getCurrentPosition()) > 2 && upStandingLeft.getCurrentPosition() < upStandingRight.getCurrentPosition() && Vel > 0){
//            upStandingLeft.setPower(Range.clip((Vel), -1.0, 1.0));
//            upStandingRight.setPower(0);
//        }else if(Math.abs(upStandingRight.getCurrentPosition() - upStandingLeft.getCurrentPosition()) > 2 && upStandingLeft.getCurrentPosition() < upStandingRight.getCurrentPosition() && Vel < 0){
//            upStandingLeft.setPower(0);
//            upStandingRight.setPower(Range.clip((-Vel), -1.0, 1.0));
//        }else {
            upStandingLeft.setPower(Range.clip((Vel), -1.0, 1.0));
            upStandingRight.setPower(Range.clip((-Vel), -1.0, 1.0));
//        }

    }

    public synchronized void setPosHorizontalTeleOp(double Pos){
        servosService.getHorizontal().setPosition(Range.clip(Pos, OPEN_POS_HORIZONTAL, CLOSE_POS_HORIZONTAL));
    }

}
