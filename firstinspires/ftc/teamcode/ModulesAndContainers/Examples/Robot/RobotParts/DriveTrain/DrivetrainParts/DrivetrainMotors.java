package org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.DriveTrain.DrivetrainParts;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Modules.Extenders.MotorModule;

public class DrivetrainMotors extends MotorModule {
    private DcMotor leftB;
    private DcMotor leftF;
    private DcMotor rightB;
    private DcMotor rightF;
    public DrivetrainMotors(OpMode op) {
        super(op);

        try {
            rightB = hardwareMap.get(DcMotor.class, "rightB");
            rightF = hardwareMap.get(DcMotor.class, "rightF");
            leftF = hardwareMap.get(DcMotor.class, "leftF");
            leftB = hardwareMap.get(DcMotor.class, "leftB");
        } catch (Exception e) {
            isInizialized = false;
            return;
        }


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
    private double curLeftBPower, curLeftFPower, curRightFPower, curRightBPower;

    public void setPower(double forwardPow, double sidePow, double anglePow){

        double minPow = 0.13 / voltageSensorClass.getkPower();

        if(voltageSensorClass.getkPower() != 0){
            forwardPow = forwardPow * 12.5 / voltageSensorClass.getCurVoltage();
            sidePow = sidePow * 12.5 / voltageSensorClass.getCurVoltage();
            anglePow = anglePow * 12.5 / voltageSensorClass.getCurVoltage();
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


    @Override
    public void showData() {
        telemetry.addLine("===DRIVETRAIN MOTORS===");
        if (isInizialized) {
            telemetry.addData("Powers right side", "RF:%.2f RB:%.2f",curRightFPower, curRightBPower);
            telemetry.addData("Powers left side", "LF:%.2f LB:%.2f",curLeftFPower, curLeftBPower);
        }else{
            telemetry.addLine("DEVICE NOT FOUND");
        }
        telemetry.addLine();
    }
}
