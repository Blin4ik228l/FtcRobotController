package org.firstinspires.ftc.teamcode.MainParts.Examples.Robot.RobotParts.HoodedShoter.Modules;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.MainParts.Examples.Robot.RobotParts.DriveTrain.DrivetrainParts.Odometry.Parts.CameraClass;
import org.firstinspires.ftc.teamcode.MainParts.Examples.Robot.RobotParts.DriveTrain.DrivetrainParts.Odometry.Parts.MathUtils.PID;

public class TurretClass extends CameraClass {
    private DcMotor turretMotor;
    private PID trackingPID, backingPID;
    public TurretClass(OpMode op) {
        super(op);

        try {
            this.turretMotor = hardwareMap.get(DcMotor.class, "m1");
        } catch (Exception e) {
            isInizialized = false;
            return;
        }

        this.turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        this.trackingPID = new PID(0.45, 0.0,0, -1, 1);
        this.backingPID = new PID(2, 0,0,-1,1);

        this.turretState = TurretState.Idle;
    }
    private TurretState turretState;
    private double lastMotorPos, curMotorPos, deltaPos;
    private double deltaAngle;
    private double target;
    private final double outPutRes = 288;
    private final double RAD = 180/Math.PI;
    public enum TurretState{
        Idle,
        Rotating,
        Back_to_zero,
        Lost
    }

    public void execute(){
        if (!isInizialized) return;
       execTurretControl();
       execFeedingSystem();
    }
    public void execTurretControl(){
        double volT = 0;
        curMotorPos = turretMotor.getCurrentPosition();
        deltaPos += lastMotorPos - curMotorPos;
        lastMotorPos = curMotorPos;

        switch (turretState){
            case Idle:
                turretState = TurretState.Rotating;
                break;
            case Rotating:

                switch (tagState){
                    case Detected:
                        target = cameraBearing;
                        deltaPos = 0;
                        break;
                    case UnDetected:
                        deltaAngle = deltaPos / outPutRes;

                        target = cameraBearing + deltaAngle;
                        break;
                }

                if(Math.abs(curMotorPos) > outPutRes) {
                    turretState = TurretState.Back_to_zero;
                    return;
                }

                if(Math.abs(target) < Math.toRadians(1.5)) {
                    target = 0;
                }

                volT = trackingPID.calculate(target);
                break;
            case Back_to_zero:
                target = -curMotorPos/outPutRes;
                if(Math.abs(target) < Math.toRadians(1.5)) {
                    deltaPos = 0;
                    turretState = TurretState.Rotating;
                    return;
                }
                volT = trackingPID.calculate(target);
                break;
            case Lost:
                break;
        }

        turretMotor.setPower(volT);
    }
    public void execFeedingSystem(){

    }

    @Override
    public void showData() {
        telemetry.addLine("===TURRET===");
        if (isInizialized) {
            telemetry.addData("Turret state", turretState.toString());
            telemetry.addData("ticks", curMotorPos);
            telemetry.addData("Pow", turretMotor.getPower());
            telemetry.addData("State", tagState.toString());
            telemetry.addData("bear", cameraBearing * RAD);
            telemetry.addData("target", target * RAD);
        }else{
            telemetry.addLine("DEVICE NOT FOUND");
        }
        telemetry.addLine();

        super.showData();
    }
}
