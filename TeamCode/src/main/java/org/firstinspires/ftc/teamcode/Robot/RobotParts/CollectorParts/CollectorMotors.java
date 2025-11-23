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

        encMotorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);//Запускаем
        encMotorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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

    public void onIntake(){
        double targetInTakePower = -1;
        double targetFlyWheelVel = 1;

        setPower(targetInTakePower, targetFlyWheelVel);
    }

    public void offIntake(){
        double targetInTakePower = 0;
        double targetFlyWheelVel = 0;

        setPower(targetInTakePower, targetFlyWheelVel);
    }

    public void reverseForAWhile(double targetTime){
        double targetInTakePower = 1;
        double targetFlyWheelVel = 0;

        if(inTakeCurPower != targetInTakePower) runTimeAll.reset();

        setPower(targetInTakePower, targetFlyWheelVel);

        if(runTimeAll.seconds() >= targetTime) offIntake();
    }

    public void offFLyWheel(){
        double targetInTakePower = 0;
        double targetFlyWheelVel = 0;

        setPower(targetInTakePower, targetFlyWheelVel);
    }

    public void onFLyWheel(){
        double targetInTakePower = 0;
        double targetFlyWheelVel = 1;

        setPower(targetInTakePower, targetFlyWheelVel);
    }

    public void setPower(double targetIntakePow, double targetFlyWheelPower){
        inTakeMotor.setPower(targetIntakePow);

        //encMotorLeft.setVelocity(flyWheelVel);
        //encMotorRight.setVelocity(-flyWheelVel);

        encMotorLeft.setPower(targetFlyWheelPower);
        encMotorRight.setPower(-targetFlyWheelPower);

        inTakeCurPower = inTakeMotor.getPower();

        curLeftVel = encMotorLeft.getVelocity(AngleUnit.RADIANS);
        curRightVel = encMotorRight.getVelocity(AngleUnit.RADIANS);

        curOverallVel = curLeftVel != 0 && curRightVel != 0 ? (curLeftVel - curRightVel) / 2.0 : curLeftVel - curRightVel;
    }

    @Override
    public void showData(){
        telemetry.addLine("===COLLECTOR MOTORS===");
        telemetry.addData("InTake Power", inTakeCurPower);
        telemetry.addData("FlyWheelSpeed overall","%.2f", curOverallVel);
        telemetry.addData("Left motor speed","%.2f", curLeftVel);
        telemetry.addData("Right motor speed","%.2f", curRightVel);
        telemetry.addData("Motors state", motorsState.toString());
        telemetry.addData("collectorMotorsState state", collectorMotorsState.toString());
        telemetry.addLine();
    }
}