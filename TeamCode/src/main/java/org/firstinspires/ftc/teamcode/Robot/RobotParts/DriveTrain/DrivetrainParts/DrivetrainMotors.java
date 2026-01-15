package org.firstinspires.ftc.teamcode.Robot.RobotParts.DriveTrain.DrivetrainParts;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Modules.Types.Module;

public class DrivetrainMotors extends Module {
    private final DcMotor leftB;
    private final DcMotor leftF;
    private final DcMotor rightB;
    private final DcMotor rightF;
    public DrivetrainMotors(OpMode op) {
        super(op.telemetry);
        rightB = op.hardwareMap.get(DcMotor.class, "rightB");
        rightF = op.hardwareMap.get(DcMotor.class, "rightF");
        leftF = op.hardwareMap.get(DcMotor.class, "leftF");
        leftB = op.hardwareMap.get(DcMotor.class, "leftB");

        rightB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightB.setDirection(DcMotorSimple.Direction.FORWARD);
        rightF.setDirection(DcMotorSimple.Direction.FORWARD);
        leftF.setDirection(DcMotorSimple.Direction.REVERSE);
        leftB.setDirection(DcMotorSimple.Direction.REVERSE);

        rightB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        telemetry.addLine("Motors on drivetrain Inited");
    }
    public DcMotor getLeftB() {
        return leftB;
    }

    public DcMotor getLeftF() {
        return leftF;
    }

    public DcMotor getRightB() {
        return rightB;
    }

    public DcMotor getRightF() {
        return rightF;
    }
    private double kPower;
    private double curLeftBPower, curLeftFPower, curRightFPower, curRightBPower;

    public void setPower(double forwardPow, double sidePow, double anglePow){
        double minPow = 0.13 / kPower;

        if(kPower != 0){
            forwardPow = forwardPow * kPower;
            sidePow = sidePow * kPower;
            anglePow = anglePow * kPower;
        }

//        if(forwardPow != 0 && Math.abs(forwardPow) < minPow) forwardPow = Math.signum(forwardPow) * minPow;
//        if(sidePow != 0 &&  Math.abs(sidePow) < minPow) sidePow = Math.signum(sidePow) * minPow;
//        if(anglePow != 0 &&  Math.abs(anglePow) < minPow) anglePow = Math.signum(anglePow) * minPow;

        rightB.setPower(forwardPow - sidePow + anglePow);
        rightF.setPower(forwardPow + sidePow + anglePow);
        leftF.setPower(forwardPow - sidePow - anglePow);
        leftB.setPower(forwardPow + sidePow - anglePow);

        curRightBPower = rightB.getPower();
        curRightFPower = rightF.getPower();
        curLeftBPower = leftB.getPower();
        curLeftFPower = leftF.getPower();
    }

    public void setKPower(double kPower){
        this.kPower = kPower;
    }

    @Override
    public void showData() {
        telemetry.addLine("===DRIVETRAIN MOTORS===");
        telemetry.addData("Powers right side", "RF:%.2f RB:%.2f",curRightFPower, curRightBPower);
        telemetry.addData("Powers left side", "LF:%.2f LB:%.2f",curLeftFPower, curLeftBPower);
        telemetry.addLine();
    }
}
