package org.firstinspires.ftc.teamcode.Robot.RobotParts.CollectorParts;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Modules.MainModule;
import org.firstinspires.ftc.teamcode.Modules.Module;

public class CollectorMotors extends Module {
    public CollectorMotors(OpMode op){
        super(op.telemetry);

        inTakeMotor = op.hardwareMap.get(DcMotor.class, "inTake");
        encMotorRight = op.hardwareMap.get(DcMotorEx.class, "flyWheelRight");
        encMotorLeft = op.hardwareMap.get(DcMotorEx.class, "flyWheelLeft");

        inTakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        encMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);// обновляем правый энкодер
        encMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);// обновляем левый энкодер

        encMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);//Запускаем
        encMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

//        encMotorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);//Запускаем
//        encMotorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        inTakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        encMotorRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        encMotorLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        runTimeAll = new ElapsedTime();

        telemetry.addLine("Motors on collector inited");
    }
    private final DcMotor inTakeMotor;
    private final DcMotorEx encMotorRight;
    private final DcMotorEx encMotorLeft;
    public DcMotor getInTakeMotor() {
        return inTakeMotor;
    }
    public DcMotorEx getEncMotorLeft() {
        return encMotorLeft;
    }
    public DcMotorEx getEncMotorRight() {
        return encMotorRight;
    }
    public double curOverallVel, curLeftVel, curRightVel, inTakeCurPower;
    public double inTakePower;
    public double flyWheelVel;
    public ElapsedTime runTimeAll;
    public CollectorMotorsState collectorMotorsState;
    public enum CollectorMotorsState{
        Ready,
        Unready
    }
    private MotorsState motorsState;
    public MotorsState targetMotorState;
    public enum MotorsState{
        OnIntake,
        OffInTake,
        ReverseForWhile,
        OnFlyWheel,
        OffFlyWheel,
    }
    public double kPower;
    public double targSpeed;
    public void onIntake(){
        double targetInTakePower = -1 * kPower;
        double targetFlyWheelVel = 1 * kPower;

        setPower(targetInTakePower);
    }

    public void offIntake(){
        double targetInTakePower = 0;
        double targetFlyWheelVel = 0;

        setPower(targetInTakePower);
    }
    public void setSpeed(double speed){

        encMotorLeft.setVelocity(speed, AngleUnit.RADIANS);
        encMotorRight.setVelocity(-speed, AngleUnit.RADIANS);

        curLeftVel = encMotorLeft.getVelocity(AngleUnit.RADIANS);
        curRightVel = encMotorRight.getVelocity(AngleUnit.RADIANS);

        curOverallVel = curLeftVel != 0 && curRightVel != 0 ? (curLeftVel + curRightVel) / 2.0 : curLeftVel + curRightVel;
    }

    public void reverseForAWhile(double targetTime){
        double targetInTakePower = 1;
        double targetFlyWheelVel = 0;

        if(inTakeCurPower != targetInTakePower) runTimeAll.reset();

        setPower(targetInTakePower);

        if(runTimeAll.seconds() >= targetTime) offIntake();
    }

    public void offFLyWheel(){
        double targetInTakePower = 0;
        double targetFlyWheelVel = 0;

        setSpeed(targetInTakePower);
    }

    public void onFLyWheel(){
        double targetInTakePower = 0;
        double targetFlyWheelVel = 1;

        setSpeed(targetInTakePower);
    }

    public void setPower(double targetIntakePow){
        inTakeMotor.setPower(targetIntakePow);

        inTakeCurPower = inTakeMotor.getPower();
    }

    public void setKPower(double kPower){
        this.kPower = kPower;
    }

    @Override
    public void showData(){
        telemetry.addLine("===COLLECTOR MOTORS===");
        telemetry.addData("InTake Power", inTakeCurPower);
        telemetry.addData("FlyWheelSpeed overall","%.2f", curOverallVel * 0.04 * 100);
        telemetry.addData("Left motor speed","%s", curLeftVel);
        telemetry.addData("Right motor speed","%s", curRightVel);
        telemetry.addData("Targ speed","%s", targSpeed);
        telemetry.addData("Left motor power","%s", encMotorLeft.getPower());
        telemetry.addData("Right motor power","%s", encMotorRight.getPower());
//        telemetry.addData("Motors state", motorsState.toString());
//        telemetry.addData("collectorMotorsState state", collectorMotorsState.toString());
        telemetry.addLine();
    }
}