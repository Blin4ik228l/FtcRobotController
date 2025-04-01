package org.firstinspires.ftc.teamcode.Game.Robot.Modules.Usable;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Game.Robot.Modules.Module;
import org.firstinspires.ftc.teamcode.Utils.Consts.Consts;
import org.firstinspires.ftc.teamcode.Utils.Consts.ConstsTeleskope;
import org.firstinspires.ftc.teamcode.Game.Players.Joystick.ClassJoystick;
import org.firstinspires.ftc.teamcode.Game.Robot.Logic.RobotModulesStatus.ComonStatuses.EncoderStatus;
import org.firstinspires.ftc.teamcode.Game.Robot.Logic.RobotModulesStatus.ComonStatuses.MotorsStatus;
import org.firstinspires.ftc.teamcode.Game.Robot.Logic.RobotModulesStatus.RobotModuleStatus;

public class TeleSkope implements Module, ConstsTeleskope {
    public final OpMode op;
    public final ServosService servosService;

    private DcMotor upStandingLeft;
    private DcMotor upStandingRight;
    private double height;
    private double oldHeight;

    public EncoderStatus leftEncUpSt;
    public EncoderStatus rightEncUpSt;

    private double leftEncUpold;
    private double rightEncUpold;

    public ElapsedTime workTime = new ElapsedTime();

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

//        height += Math.ceil((deltaRightEncUp + deltaLeftEncUp) / 2.0);

        height += deltaLeftEncUp;
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

    public void setTeleskopeAuto(double speed, double targetHeight){
        oldHeight = height;
        calculateHeight();

        double targetVel = speed * Math.signum(targetHeight - height);
        setVelUpStandingTeleOp(targetVel);
    }

    public void setTele(double power, double targetHeight){
        calculateHeight();

        double targetPower = power * Math.signum(targetHeight - height);
        setVelUpStandingTeleOp(targetPower);
    }
    public synchronized void setTeleskope(double vel, double deltaPos){
        calculateHeight();

        setVelUpStandingTeleOp(vel);
        setLeftRightHorizont(deltaPos);
    }
    public synchronized void setTeleskope2(double vel, double deltaPos){
        if(vel != 0) ;
        setVelUpStandingTeleOp(vel);
        setLeftRightHorizont(deltaPos);
    }

    public void setTeleskopeHeight(double targetHeight, ClassJoystick joystick){
        calculateHeight();
        if(!joystick.isBack_Button) {
            while (Math.abs(height - targetHeight) > 2.5 && !joystick.isBack_Button) {
                calculateHeight();
                double targetVel = 1 * Math.signum(targetHeight - height);
                setVelUpStandingTeleOp(targetVel);
            }
        }
    }

    public void setTeleskopeHeightAuto(double targetHeight, double max_speed){
        calculateHeight();
        while (Math.abs(height - targetHeight) > 2.5) {
            calculateHeight();
            double targetVel = max_speed * Math.signum(targetHeight - height);
            setVelUpStandingTeleOp(targetVel);
        }
    }


    public synchronized void setTeleskopeProp(double vel, double Pos){
        calculateHeight();

        double DEAD_ZONE_HEIGHT = 106;

        double PROPRTIONAL_HEIGHT = 15;// Высота на которой телескопы будут двигаться одновременно

        double P = (CLOSE_POS_HORIZONTAL - 0.2)/(DEAD_ZONE_HEIGHT - PROPRTIONAL_HEIGHT);

        double propLen = CLOSE_POS_HORIZONTAL - (height - PROPRTIONAL_HEIGHT) * P;

        if((height > PROPRTIONAL_HEIGHT) ) {
            setVelUpStandingTeleOp(vel);
        }else{
            setVelUpStandingTeleOp(vel);
        }

    }
    public void setSmallTele(ServosService.servoPos servoPos, double Pos){
        if (servoPos == ServosService.servoPos.UP){
            servosService.setLeftStartPos();
            servosService.setRightStartPos();
        }
        if (servoPos == ServosService.servoPos.DOWN) {
            servosService.getLeft().setPosition(Pos);
            servosService.getRight().setPosition(1 - Pos);
        }

    }

    public void setLeftRightHorizont(double delta){
        servosService.getLeft().setPosition(Range.clip(CLOSE_POS_HORIZ_LEFT + delta,CLOSE_POS_HORIZ_LEFT, OPEN_POS_HORIZ_LEFT));
        servosService.getRight().setPosition(Range.clip(CLOSE_POS_HORIZ_RIGHT - delta,OPEN_POS_HIRIZ_RIGHT, CLOSE_POS_HORIZ_RIGHT));
    }

    public synchronized void setVelUpStandingTeleOp(double Vel){
        upStandingLeft.setPower(Range.clip((Vel), -1.0, 1.0));
        upStandingRight.setPower(Range.clip((-Vel), -1.0, 1.0));
    }

    public synchronized void setPosHorizontalTeleOp(double Pos){
        servosService.getHorizontal().setPosition(Range.clip(Pos, OPEN_POS_HORIZONTAL, CLOSE_POS_HORIZONTAL));
    }

    public void getTicks(){
        op.telemetry.addLine("TicksFrom telesopes")
                .addData("\nleftVert", upStandingLeft.getCurrentPosition())
                .addData("\nrightVert", upStandingRight.getCurrentPosition());
        op.telemetry.addLine();
    }

}
