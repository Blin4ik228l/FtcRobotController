package org.firstinspires.ftc.teamcode.Robot.RobotParts.CollectorParts.AutomaticParts;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Modules.Module;

public class MotorsOnCollector extends Module {
    public MotorsOnCollector(OpMode op){
        super(op.telemetry);

        inTake = op.hardwareMap.get(DcMotor.class, "inTake");
        flyWheelRight = op.hardwareMap.get(DcMotor.class, "flyWheelRight");
        flyWheelLeft = op.hardwareMap.get(DcMotor.class, "flyWheelLeft");

        inTake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flyWheelRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flyWheelLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        inTake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flyWheelRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flyWheelLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        selfData = new SelfData();

        telemetry.addLine("Motors on collector inited");
    }
    private final DcMotor inTake;
    private final DcMotor flyWheelRight;
    private final DcMotor flyWheelLeft;
    private final SelfData selfData;

    public DcMotor getFlyWheelLeft() {
        return flyWheelLeft;
    }

    public DcMotor getFlyWheelRight() {
        return flyWheelRight;
    }

    public DcMotor getInTake() {
        return inTake;
    }

    public SelfData getSelfData() {
        return selfData;
    }

    public void turnOnInTake(boolean bol){
        if(bol) inTake.setPower(-1);
        else inTake.setPower(0);
    }
    public void turnOnFlyWheel(boolean bol){
        if(bol){
            flyWheelLeft.setPower(-1);
            flyWheelRight.setPower(1);
        }else{
            flyWheelLeft.setPower(0);
            flyWheelRight.setPower(0);
        }
    }
    public void setSpeedOnFlyWheel(double contTicks){

    }

    public class SelfData{

    }
    double getAngle(double length){
        double velMahovik = (6000 * 8 * Math.PI) / 60;

        double velBall = 2500;
        //580

        return ((length * 981) / (Math.pow(velBall, 2)));
    }

    double getLength(double angle){
        double velMahovik = (6000 * 8 * Math.PI) / 60;

        double velBall = 2500;

        return (Math.pow(velBall, 2) * Math.sin(angle * 2)) / 981;
    }
}