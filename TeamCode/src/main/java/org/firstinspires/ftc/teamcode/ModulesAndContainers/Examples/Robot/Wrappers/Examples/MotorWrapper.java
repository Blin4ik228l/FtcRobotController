package org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.Wrappers.Examples;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.HardwareBuilder;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.VoltageSensorClass;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.Wrappers.Extenders.DeviceWrapper;

public class MotorWrapper extends DeviceWrapper {
    private DcMotor motor;
    private MotorConfigurationType motorConfigurationType;
    private ElapsedTime signalTime;
    private VoltageSensorClass voltageSensorClass;

    public MotorWrapper(OpMode op, String deviceName) {
        super(op);
        this.deviceName = deviceName;
        try {
            motor = hardwareMap.get(DcMotor.class, deviceName);

            if (motor instanceof DcMotorEx){
                motor = (DcMotorEx) motor;
            }

            motorConfigurationType = motor.getMotorType();
        }catch (Exception e){
            isInitialized = false;
        }

        this.signalTime = new ElapsedTime();
        sayInited();
    }
    private double voltageCompensation;
    private double targetVol;
    public void setPower(double power){
        if(!isInitialized) return;

        if (motor.getPower() != power) signalTime.reset();

        double currentVoltage = voltageSensorClass.getCurVoltage();
        if (currentVoltage <= 0) currentVoltage = targetVol;

        double voltageMultiplier = targetVol / currentVoltage;

        power *= (voltageCompensation * voltageMultiplier);
        motor.setPower(power);
    }
    public double getPower(){
        return motor.getPower();
    }

    public boolean isBusy(double delayTime){
        return signalTime.seconds() > delayTime;
    }
    public DcMotorEx getMotorEx(){
        if(motor instanceof DcMotorEx) return (DcMotorEx) motor;
        else return null;
    }
    public DcMotor getMotor(){
        if(motor instanceof DcMotor) return motor;
        else return null;
    }
    public MotorConfigurationType getMotorConfigurationType(){
        return motorConfigurationType;
    }
    @Override
    public void showData() {
        if(!isInitialized) telemetry.addLine( deviceName + " " + "Not Found/Attached");
        else {
            telemetry.addData(deviceName, getMotor().getPower());
        }
    }
    public static class Builder extends HardwareBuilder{
        private MotorWrapper motorWrapper;

        @Override
        public Builder initialize(OpMode op, String deviceName) {
            motorWrapper = new MotorWrapper(op, deviceName);
            return this;
        }
        public Builder setFields(VoltageSensorClass voltageSensorClass, double targetVol,double voltageComp){
            motorWrapper.voltageSensorClass = voltageSensorClass;
            motorWrapper.voltageCompensation = voltageComp;
            motorWrapper.targetVol = targetVol;
            return this;
        }
        public Builder setDirection(DcMotorSimple.Direction direction){
            motorWrapper.motor.setDirection(direction);
            return this;
        }
        public Builder setMode(DcMotor.RunMode runMode){
            motorWrapper.motor.setMode(runMode);
            return this;
        }
        public Builder setBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior){
            motorWrapper.motor.setZeroPowerBehavior(zeroPowerBehavior);
            return this;
        }

        public MotorWrapper get(){
            return motorWrapper;
        }
    }
}
