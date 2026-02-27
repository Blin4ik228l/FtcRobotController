package org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.Wrappers.Examples;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.Wrappers.HardwareBuilder;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.VoltageSensorClass;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.Wrappers.Extenders.DeviceWrapper;

public class MotorWrapper extends DeviceWrapper {
    private DcMotor motor;
    private MotorConfigurationType motorConfigurationType;
    private ElapsedTime signalTime;
    private VoltageSensorClass voltageSensorClass;

    public MotorWrapper(OpMode op, String deviceName) {
        super(op);
        this.searchingDevice = deviceName;
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
        return signalTime.seconds() < delayTime;
    }
    public DcMotorEx getMotorEx(){
        if(motor instanceof DcMotorEx) return (DcMotorEx) motor;
        else return null;
    }
    public DcMotor getMotor(){
       return motor;
    }
    public MotorConfigurationType getMotorConfigurationType(){
        return motorConfigurationType;
    }

    @Override
    public void showDataExt() {
        telemetry.addLine("===" + searchingDevice + "===");
        telemetry.addData("Power", getMotor().getPower());
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
            if(!motorWrapper.isInitialized) return this;
            motorWrapper.motor.setDirection(direction);
            return this;
        }
        public Builder setMode(DcMotor.RunMode runMode){
            if(!motorWrapper.isInitialized) return this;
            motorWrapper.motor.setMode(runMode);
            return this;
        }
        public Builder setBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior){
            if(!motorWrapper.isInitialized) return this;
            motorWrapper.motor.setZeroPowerBehavior(zeroPowerBehavior);
            return this;
        }

        public MotorWrapper get(){
            return motorWrapper;
        }
    }
}
