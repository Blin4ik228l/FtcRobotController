package org.firstinspires.ftc.teamcode.Robot.RobotParts.CollectorParts;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Modules.Types.Module;

public class CollectorMotors extends Module {
    public CollectorMotors(OpMode op){
        super(op.telemetry);

        inTakeMotor = op.hardwareMap.get(DcMotor.class, "inTake");
        encMotorRight = op.hardwareMap.get(DcMotorEx.class, "flyWheelRight");
        encMotorLeft = op.hardwareMap.get(DcMotorEx.class, "flyWheelLeft");

        inTakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        encMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);// обновляем правый энкодер
        encMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);// обновляем левый энкодер

//        encMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);//Запускаем
//        encMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        encMotorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);//Запускаем
        encMotorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


//        encMotorLeft.setVelocityPIDFCoefficients(P, I, D, F);
//        encMotorRight.setVelocityPIDFCoefficients(P, I, D, F);

        inTakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        encMotorRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        encMotorLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);


        runTimeFlyWheel = new ElapsedTime();
        runTimeIntake = new ElapsedTime();

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
    public PIDFCoefficients pidfCoefficients;

    public double curOverallVel, curLeftVel, curRightVel, inTakeCurPower, curOverallInMeters;
    public ElapsedTime runTimeFlyWheel, runTimeIntake;
    public double kPower;
    public double targSpeedInRad;
    public FlyWheelStates flyWheelStates = FlyWheelStates.Unready;

//    private double P = 18, I = 0.15, D = 3.0, F = 0.45;
private double P = 18, I = 0, D = 0, F = 0.45;

    public enum FlyWheelStates{
        Ready,
        Unready
    }

    public void setPower(double targetIntakePow){
        if(Math.abs(inTakeCurPower - targetIntakePow) < 0.1) return;

        inTakeMotor.setPower(targetIntakePow);

        inTakeCurPower = inTakeMotor.getPower();

        runTimeIntake.reset();
    }
    double koef = 0.98;
    double i =0;
    int attempts;
    public void setSpeed(double speed, double pow){
        if(speed == 0) {
            attempts = 0;
            i = 0;}
        targSpeedInRad = speed ;
/// MAX_RAD_SPEED * MAX_EXPERIMENTAL_SPEED_IN_METERS
//        encMotorLeft.setVelocity(speed, AngleUnit.RADIANS);
//        encMotorRight.setVelocity(-speed, AngleUnit.RADIANS);

        encMotorLeft.setPower(pow * (koef + i) );
        encMotorRight.setPower(-pow * (koef + i));

        calcCurSpeed();

        double errorSpeed = targSpeedInRad - curOverallVel;

        if(attempts > 20){
            i += errorSpeed / 700;
        }
        flyWheelStates = Math.abs(curOverallVel / targSpeedInRad - 1) < 0.03 ? CollectorMotors.FlyWheelStates.Ready : CollectorMotors.FlyWheelStates.Unready;

        if(flyWheelStates == FlyWheelStates.Unready) runTimeFlyWheel.reset();

        attempts++;
    }

    public void setPIDF(double P, double I, double D, double F){
        encMotorLeft.setVelocityPIDFCoefficients(P, I, D, F);
        encMotorRight.setVelocityPIDFCoefficients(P, I, D, F);
    }

    public void calcCurSpeed(){
        curLeftVel = encMotorLeft.getVelocity(AngleUnit.RADIANS);
        curRightVel = encMotorRight.getVelocity(AngleUnit.RADIANS);

        curOverallVel = curLeftVel != 0 && curRightVel != 0 ? (curLeftVel + curRightVel) / 2.0 : curLeftVel + curRightVel;

        curOverallInMeters = curOverallVel / MAX_RAD_SPEED * MAX_EXPERIMENTAL_SPEED_IN_METERS;
    }

    public void onIntake(){
        double targetInTakePower = -1 * kPower;

        setPower(targetInTakePower);
    }
    public void reverseInTake(){
        double targetInTakePower = 1 * kPower;

        setPower(targetInTakePower);
    }

    public void offIntake(){
        double targetInTakePower = 0;

        setPower(targetInTakePower);
    }
    public void offFLyWheel(){
        double targetFlyWheelSpeed = 0;

        setSpeed(targetFlyWheelSpeed, 0);
    }
    public void preFireSpeedFlyWheel(){
        double targetFlyWheelSpeed = 4;

        setSpeed(targetFlyWheelSpeed, 1);
    }
    public void setKPower(double kPower){
        this.kPower = kPower;
    }

    @Override
    public void showData(){
        telemetry.addLine("===COLLECTOR MOTORS===");
//        telemetry.addData("FlyWheelSpeed overall in rad","%.2f /s", curOverallVel);
        telemetry.addData("PIDF", "P %s I %s D %s F %s", encMotorRight.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).p,
                encMotorRight.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).i, encMotorRight.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).d,
                encMotorRight.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).f);
        telemetry.addData("Targ speed in rad","%.2f /s", targSpeedInRad);
        telemetry.addData("FlyWheelSpeed overall in rad","%.2f /s", curOverallVel);
        telemetry.addData("errror%", Math.abs(curOverallVel / targSpeedInRad - 1) * 100);
//        telemetry.addData("Left motor speed in rad","%.2f /s", curLeftVel);
//        telemetry.addData("Right motor speed in rad","%.2f /s", curRightVel);
        telemetry.addData("InTake Power", inTakeCurPower);
        telemetry.addData("Run time intake", runTimeIntake);
        telemetry.addData("Run time flywhell", runTimeFlyWheel);
        telemetry.addLine();
    }
}