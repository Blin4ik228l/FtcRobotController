package org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.Wrappers;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ModulesAndContainers.Modules.Module;

import java.util.HashMap;

public class MotorWrapper extends Module {
    private DcMotorEx motor;
    private ElapsedTime signalTime;
    public MotorWrapper(OpMode op, DcMotorEx dcMotor) {
        super(op);
        this.motor = dcMotor;
        this.signalTime = new ElapsedTime();
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
    public DcMotorEx getMotor(){
        return motor;
    }
    @Override
    public void showData() {

    }
    public static class Builder {
        private HashMap<String, MotorWrapper> motors = new HashMap<>();

        public Builder add(OpMode op, DcMotorEx dcMotor) {
            MotorWrapper motorWrapper = new MotorWrapper(op, dcMotor);
            motors.put(motorWrapper.motor.getDeviceName(), motorWrapper);
            return this;
        }
        public MotorWrapper get(String deviceName){
            return motors.get(deviceName);
        }
    }
}
