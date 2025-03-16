package org.firstinspires.ftc.teamcode.RobotCore.RobotMainModules.Physical.Groups;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Consts.Consts;
import org.firstinspires.ftc.teamcode.Consts.ConstsTeleskope;
import org.firstinspires.ftc.teamcode.RobotCore.RobotMainModules.Module;
import org.firstinspires.ftc.teamcode.RobotCore.RobotModulesStatus.ComonStatuses.EncoderStatus;
import org.firstinspires.ftc.teamcode.RobotCore.RobotModulesStatus.ComonStatuses.MotorsStatus;
import org.firstinspires.ftc.teamcode.RobotCore.RobotModulesStatus.RobotModuleStatus;

public class TeleSkope implements Module, ConstsTeleskope {
    public final OpMode op;
    public final ServosService servosService;

    private DcMotor upStandingLeft;
    private DcMotor upStandingRight;
    private double height;

    public EncoderStatus leftEncUpSt;
    public EncoderStatus rightEncUpSt;

    private double leftEncUpold;
    private double rightEncUpold;

    boolean completed = false;
    public RobotModuleStatus motorsTeleskopeSt;


    public TeleSkope(OpMode op, ServosService servosService){
        this.op = op;
        this.servosService = servosService;
    }
    @Override
    public void init() {
        leftEncUpSt = EncoderStatus.ZeroDelta;
        rightEncUpSt = EncoderStatus.ZeroDelta;
        motorsTeleskopeSt = RobotModuleStatus.Normal;

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

    public void keepInPower(){
        upStandingLeft.setPower(0.05);
        upStandingRight.setPower(0.05);
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
        return ticks / Consts.TICK_PER_CM_BARABAN;
    }

    public double cmToTicks(double cm){
        return cm * Consts.TICK_PER_CM_BARABAN;
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
        if(Pos == servosService.getHook().getPosition()){
            return;
        }
        servosService.getHook().setPosition(Pos);
    }

    public synchronized void setFlip(double Pos){
        if(Pos == servosService.getFlip().getPosition()){
            return;
        }

        servosService.getFlip().setPosition(Pos);
    }

    public synchronized MotorsStatus setTeleskopeAuto(double power, double targetHeight){
        calculateHeight();

        double targetPower= power * Math.signum(targetHeight - height);

        if(motorsTeleskopeSt == RobotModuleStatus.Normal) setVelUpStandingTeleOp(targetPower);
        else offMotors();

        return power == 0 ? MotorsStatus.Stopped : MotorsStatus.Powered;
    }

    public synchronized void setTeleskopeTele(double vel, double deltaPos, Joysticks joysticks){
        calculateHeight();
        if(vel == 0 && deltaPos == 0){
            keepInPower();
            return;
        }

        if(vel != 0) joysticks.isBack_g2 = true;
        setVelUpStandingTeleOp(vel);
        setLeftRightHorizont(deltaPos);
    }

    public void setTeleskopeHeight(double targetHeight, Joysticks joysticks){
        calculateHeight();
        if(!joysticks.isBack_G2()) {
            while (Math.abs(height - targetHeight) > 2.5 && !joysticks.isBack_G2()) {
                calculateHeight();
                double targetVel = 1 * Math.signum(targetHeight - height);
                setVelUpStandingTeleOp(targetVel);
            }
        }
    }

    public void setLeftRightHorizont(double delta){
        if(delta == 0){
            return;
        }
        servosService.getLeft().setPosition(Range.clip(CLOSE_POS_HORIZ_LEFT + delta,CLOSE_POS_HORIZ_LEFT, OPEN_POS_HORIZ_LEFT));
        servosService.getRight().setPosition(Range.clip(CLOSE_POS_HORIZ_RIGHT - delta,OPEN_POS_HIRIZ_RIGHT, CLOSE_POS_HORIZ_RIGHT));
    }

    public synchronized void setVelUpStandingTeleOp(double Vel){
        if(Vel == 0 ){
            keepInPower();
            return;
        }

        upStandingLeft.setPower(Range.clip((Vel), -1.0, 1.0));
        upStandingRight.setPower(Range.clip((-Vel), -1.0, 1.0));
    }

}
