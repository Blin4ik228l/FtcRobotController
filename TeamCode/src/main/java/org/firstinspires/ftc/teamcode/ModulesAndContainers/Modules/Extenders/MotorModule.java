package org.firstinspires.ftc.teamcode.ModulesAndContainers.Modules.Extenders;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.HardwareBuilder;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.Wrappers.Examples.MotorWrapper;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Modules.Module;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.VoltageSensorClass;

import java.util.HashMap;

public abstract class MotorModule extends Module {
    protected Builder motorsWrapper;
    public MotorWrapper.Builder motorBuilder;
    public MotorModule(OpMode op) {
        super(op);

        motorsWrapper = new Builder();
        motorBuilder = new MotorWrapper.Builder();
    }
    public class Builder{
        private HashMap<String, MotorWrapper> motors = new HashMap<>();

        public Builder add(OpMode op,MotorWrapper motorWrapper) {
            motors.put(motorWrapper.getMotor().getDeviceName(), motorWrapper);
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
