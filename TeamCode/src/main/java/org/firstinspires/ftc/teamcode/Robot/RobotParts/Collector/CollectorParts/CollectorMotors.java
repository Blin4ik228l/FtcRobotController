package org.firstinspires.ftc.teamcode.Robot.RobotParts.Collector.CollectorParts;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Modules.Types.Module;
import org.firstinspires.ftc.teamcode.Robot.RobotParts.DriveTrain.DrivetrainParts.Odometry.Parts.MathUtils.PID;

public class CollectorMotors extends Module {
    private final DcMotor inTakeMotor;
    private final DcMotorEx motorRight;
    private final DcMotorEx motorLeft;
    private Units units;
    private ControlMode controlMode;
    private ElapsedTime runTimeFlyWheel, runTimeIntake, innerTime;
    public FlyWheelStates flyWheelStates;
    public CollectorMotors(OpMode op){
        super(op);

        inTakeMotor = hardwareMap.get(DcMotor.class, "inTake");
        motorRight = hardwareMap.get(DcMotorEx.class, "flyWheelRight");
        motorLeft = hardwareMap.get(DcMotorEx.class, "flyWheelLeft");

        inTakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);// обновляем правый энкодер
        motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);// обновляем левый энкодер

        inTakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        motorLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        flyWheelStates = FlyWheelStates.Unready;

        runTimeFlyWheel = new ElapsedTime();
        runTimeIntake = new ElapsedTime();
        innerTime = new ElapsedTime();

        telemetry.addLine("Motors on collector inited");
    }

    public DcMotor getInTakeMotor() {
        return inTakeMotor;
    }
    public DcMotorEx getMotorLeft() {
        return motorLeft;
    }
    public DcMotorEx getMotorRight() {
        return motorRight;
    }
    public double curVel, curLeftVel, curRightVel, inTakeCurPower;
    private double kPower;
    public double targSpeed;

//    private double P = 18, I = 0.15, D = 3.0, F = 0.45;
//    private double  P = 30, I = 0.04, D = 0, F = 1;
//    private double  P = 11, I = 5, D = 1, F = 0;
//    private double  P = 5.64, I = 4.512, D = 1.7625, F = 0.1;
//    private double  P = 4.7, I = 5.8, D = 0.99, F = 0.086;

    private double  P = 0.8, I = 0.03, D = 0, F = 14;
    private double pG ,iG ,dG , fG;
    private double errorPart;
    double filteredLeftVel = 0;
    double filteredRightVel = 0;
    double alpha = 0.3;
    private final double p = 0.95;
    private double i = 0;
    private int attempts;
    public enum FlyWheelStates{
        Ready,
        Unready
    }
    public enum Units{
        Meters_in_sec,
        Rad_in_sec
    }
    public enum ControlMode{
        By_power,
        By_speed
    }

    public ElapsedTime getRunTimeFlyWheel() {
        return runTimeFlyWheel;
    }

    public ElapsedTime getRunTimeIntake() {
        return runTimeIntake;
    }
    public double[] getPIDF(){
        return new double[]{P, I, D, F};
    }

    public void setPIDF(double P, double I, double D, double F){
        switch (controlMode){
            case By_power:
                PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, I, D, F);
                motorLeft.setPIDFCoefficients(DcMotor.RunMode.RUN_WITHOUT_ENCODER, pidfCoefficients);
                motorRight.setPIDFCoefficients(DcMotor.RunMode.RUN_WITHOUT_ENCODER, pidfCoefficients);
                break;
            case By_speed:
                motorLeft.setVelocityPIDFCoefficients(P, I, D, F);
                motorRight.setVelocityPIDFCoefficients(P, I, D, F);
                break;
        }
    }
    public void resetPIDF(){
        P = FLYWHEEL[0];
        I = FLYWHEEL[1];
        D = FLYWHEEL[2];
        F = FLYWHEEL[3];
    }
    public void setKPower(double kPower){
        this.kPower = kPower;
    }
    public void setPower(double targetIntakePow){
        if(Math.abs(inTakeCurPower - targetIntakePow) < 0.1) return;

        inTakeMotor.setPower(targetIntakePow);

        inTakeCurPower = inTakeMotor.getPower();

        runTimeIntake.reset();
    }

    public void setPreferences(ControlMode controlMode, Units units){
        this.units = units;
        this.controlMode = controlMode;

        switch (controlMode){
            case By_power:
                motorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);//Запускаем
                motorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                break;
            case By_speed:
                motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);//Запускаем
                motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                motorLeft.setVelocityPIDFCoefficients(P, I, D, F);
                motorRight.setVelocityPIDFCoefficients(P, I, D, F);
                break;
        }
    }
    public void setSpeedFlyWheel(double speed){
        targSpeed = speed * 19.2;
        if (targSpeed == 0) innerTime.reset();

        switch (controlMode){
            case By_speed:
                motorLeft.setVelocity(speed, AngleUnit.RADIANS);
                motorRight.setVelocity(-speed, AngleUnit.RADIANS);

                pG = motorRight.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).p;
                iG = motorRight.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).i;
                dG = motorRight.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).d;
                fG = motorRight.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).f;

                calcCurSpeed();
                break;
            case By_power:
                setPowerFlyWheel(speed / MAX_RAD_SPEED);
                break;
        }

        checkReadiness();
    }

    public void setPowerFlyWheel(double pow){
        if (pow == 0)
        {
            attempts = 0;
            i = 0;
        }

//        motorLeft.setPower(pow * (p + i) );
//        motorRight.setPower(-pow * (p + i));

        motorLeft.setPower(0.5);
        motorRight.setPower(-0.5);

        calcCurSpeed();

        double error = targSpeed - curVel;

//        if(attempts > 10){
//
//        }
//        i += error / 1000;

        attempts++;
    }
    public void calcCurSpeed(){
        curLeftVel = motorLeft.getVelocity(AngleUnit.RADIANS) * 19.2;
        curRightVel = motorRight.getVelocity(AngleUnit.RADIANS) * 19.2;

        filteredLeftVel  = alpha * filteredLeftVel  + (1 - alpha) * curLeftVel;
        filteredRightVel = alpha * filteredRightVel + (1 - alpha) * curRightVel;


        curVel = filteredLeftVel != 0 && filteredRightVel != 0 ? (filteredLeftVel + filteredRightVel) / 2.0 : filteredLeftVel + filteredRightVel;

        if (units == Units.Meters_in_sec)
        {
            targSpeed = targSpeed / MAX_RAD_SPEED * MAX_EXPERIMENTAL_SPEED_IN_METERS;
            curVel = curVel / MAX_RAD_SPEED * MAX_EXPERIMENTAL_SPEED_IN_METERS;
        }
    }
    public void checkReadiness(){
        errorPart = Math.abs(curVel / targSpeed - 1);

        if (errorPart > 0.02 || curVel < targSpeed || innerTime.seconds() < 1)
        {
            flyWheelStates = FlyWheelStates.Unready;
            runTimeFlyWheel.reset();
        }
        else {
            flyWheelStates = FlyWheelStates.Ready;
        }
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

        setSpeedFlyWheel(targetFlyWheelSpeed);
    }
    public void preFireSpeedFlyWheel(){
        double targetFlyWheelSpeed = 2;

        setSpeedFlyWheel(targetFlyWheelSpeed);
    }

    @Override
    public void showData(){
        telemetry.addLine("===COLLECTOR MOTORS===");
        telemetry.addData("Units", units.toString());
        telemetry.addData("Targ","%.2f", targSpeed);
        telemetry.addData("Cur","%.2f", curVel);
        telemetry.addData("error", "Proc %.2f", errorPart * 100);
        telemetry.addData("PIDF", "P %s I %s D %s F %s",pG, iG, dG, fG);
        telemetry.addData("Pow", "L %.2f R %.2f", motorLeft.getPower(), motorRight.getPower());
//        telemetry.addData("LM","%.2f /s", curLeftVel);
//        telemetry.addData("RM","%.2f /s", curRightVel);
//        telemetry.addData("InTake Power", inTakeCurPower);
//        telemetry.addData("Run time intake", runTimeIntake);
//        telemetry.addData("Run time flywhell", runTimeFlyWheel);
        telemetry.addLine();
    }
}