package org.firstinspires.ftc.teamcode.Robot.RobotParts.CollectorParts;

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

        encoders = new EncodersInMotors(op);

        telemetry.addLine("Motors on collector inited");
    }
    private final DcMotor inTake;
    private final DcMotor flyWheelRight;
    private final DcMotor flyWheelLeft;

    public final EncodersInMotors encoders;
    public DcMotor getFlyWheelLeft() {
        return flyWheelLeft;
    }
    public DcMotor getFlyWheelRight() {
        return flyWheelRight;
    }
    public DcMotor getInTake() {
        return inTake;
    }

    public void turnOnInTake(double pow){
        inTake.setPower(pow);
    }
    public void turnOnFlyWheel(int pow){
        flyWheelLeft.setPower(-1 * pow);
        flyWheelRight.setPower(1 * pow);
    }
    public void setSpeedOnFlyWheel(double radians){
        encoders.setVelocities(radians);
    }

}