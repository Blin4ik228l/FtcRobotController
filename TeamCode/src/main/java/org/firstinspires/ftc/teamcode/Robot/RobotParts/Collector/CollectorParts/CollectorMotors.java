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

    private double  P = FLYWHEEL[0], I = FLYWHEEL[1], D = FLYWHEEL[2], F = FLYWHEEL[3];
    private double pG ,iG ,dG , fG;
    private double errorPart;
    double filteredLeftVel = 0;
    double filteredRightVel = 0;
    double alpha = 0.3;
    double lastPow;
    boolean flag = true;
    int stopCount;
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
                this.P = P;
                this.I = I;
                this.D = D;
                this.F = F;
//                PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, I, D, F);
//                motorLeft.setPIDFCoefficients(DcMotor.RunMode.RUN_WITHOUT_ENCODER, pidfCoefficients);
//                motorRight.setPIDFCoefficients(DcMotor.RunMode.RUN_WITHOUT_ENCODER, pidfCoefficients);
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
                setPowerFlyWheel(speed);
                break;
        }

        checkReadiness();
    }

    public void setPowerFlyWheel(double speed){
        voltageSensor.update();
        calcCurSpeed();

        double feedforward = F * speed;

        double error = speed - curVel / 19.2;
        double pidTerm = P * error;

        double targetVoltage = feedforward + pidTerm;

        double power = targetVoltage / voltageSensor.getCurVoltage();

        power = Math.max(-1.0, Math.min(1.0, power));

        if (speed == 0) {
            if(flag){
                lastPow = Math.abs(power);
                flag = false;
            }
            switch (stopCount){
                case 0:
                    power = 0.95 * lastPow;
                    stopCount++;
                    break;
                case 1:
                    power = 0.87 * lastPow;
                    stopCount++;
                    break;
                case 2:
                    power = 0.8 * lastPow;
                    stopCount++;
                    break;
                case 3:
                    power = 0.73 * lastPow;
                    stopCount++;
                    break;
                case 4:
                    power = 0.65 * lastPow;
                    stopCount++;
                    break;
                case 5:
                    power = 0.55 * lastPow;
                    stopCount++;
                    break;
                case 6:
                    power = 0.45 * lastPow;
                    stopCount++;
                    break;
                case 7:
                    power = 0.35 * lastPow;
                    stopCount++;
                    break;
                case 8:
                    power = 0.25 * lastPow;
                    stopCount++;
                    break;
                case 9:
                    power = 0.20 * lastPow;
                    stopCount++;
                    break;
                case 10:
                    power = 0.15 * lastPow;
                    stopCount++;
                    break;
                case 11:
                    power = 0;
                    break;
            }
        }else
        {
            stopCount = 0;
            flag = true;
        }

        motorLeft.setPower(power);
        motorRight.setPower(-power);
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

        if (errorPart > 0.01 )
        {
            flyWheelStates = FlyWheelStates.Unready;
            runTimeFlyWheel.reset();
        }
        else {
            if(runTimeFlyWheel.seconds() >= 0.1) flyWheelStates = FlyWheelStates.Ready;
        }
    }

    public void onIntake(){
        voltageSensor.update();

        double targetInTakePower = -1 * voltageSensor.getkPower();

        setPower(targetInTakePower);
    }
    public void reverseInTake(){
        voltageSensor.update();
        double targetInTakePower = 1 * voltageSensor.getkPower();

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
        telemetry.addData("PIDF", "P %s I %s D %s F %s", P, I, D, F);
        telemetry.addData("Pow", "L %.2f R %.2f", motorLeft.getPower(), motorRight.getPower());
//        telemetry.addData("LM","%.2f /s", curLeftVel);
//        telemetry.addData("RM","%.2f /s", curRightVel);
//        telemetry.addData("InTake Power", inTakeCurPower);
//        telemetry.addData("Run time intake", runTimeIntake);
//        telemetry.addData("Run time flywhell", runTimeFlyWheel);
        telemetry.addLine();
    }
}