package org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.Wrappers;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ModulesAndContainers.Modules.Extenders.MotorModule;

import java.util.HashMap;

public class MotorWrapper extends DeviceWrapper {
    private DcMotorSimple motor;
    private ElapsedTime signalTime;

    public MotorWrapper(OpMode op, MotorModule.MotorBuilder motorBuilder) {
        super(op);
        this.motor = motorBuilder.get();
        isInitialized = motorBuilder.isInit;
        deviceName = motorBuilder.deviceName;
        this.signalTime = new ElapsedTime();
        sayInited();
    }
    public void setPower(double power){
        if (motor.getPower() != power) signalTime.reset();
        motor.setPower(power);
    }
    public double getPower(){
        return motor.getPower();
    }

    public boolean isBusy(double delayTime){
        return signalTime.seconds() > delayTime;
    }
    public DcMotorEx getMotorEx(){
        return (DcMotorEx) motor;
    }
    public DcMotor getMotor(){
        return (DcMotor) motor;
    }
    @Override
    public void showData() {
        if(!isInitialized) telemetry.addLine( deviceName + " " + "Not Found/Attached");
        else {
            telemetry.addData(deviceName, getMotor().getPower());
        }
    }
    public static class Builder {
        private HashMap<String, MotorWrapper> motors = new HashMap<>();

        public Builder add(OpMode op, MotorModule.MotorBuilder motorBuilder) {
            MotorWrapper motorWrapper = new MotorWrapper(op, motorBuilder);
            motors.put(motorBuilder.deviceName, motorWrapper);
            return this;
        }
        public MotorWrapper get(String deviceName){
            return motors.get(deviceName);
        }
        public void showData(){
            for (MotorWrapper motor:motors.values()) {
                motor.showData();
            }
        }
    }
}
