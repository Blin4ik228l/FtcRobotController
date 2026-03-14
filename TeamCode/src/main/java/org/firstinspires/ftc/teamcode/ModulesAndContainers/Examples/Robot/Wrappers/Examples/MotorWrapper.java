package org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.Wrappers.Examples;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.Config.MainFile;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.VoltageSensorClass;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Modules.Extenders.Extenders2.ExecutableModule;

public class MotorWrapper extends ExecutableModule {
    private DcMotor motor;
    private MotorConfigurationType motorConfigurationType;
    private static VoltageSensorClass voltageSensorClass;
    private ElapsedTime signalTime;

    public MotorWrapper(MainFile mainFile, String searchingDevice) {
        super(mainFile, searchingDevice);

        try {
            motor = hardwareMap.get(DcMotor.class, searchingDevice);

            if (motor instanceof DcMotorEx){
                motor = (DcMotorEx) motor;
            }
            voltageSensorClass = mainFile.voltageSensorClass;
            motorConfigurationType = motor.getMotorType();
        }catch (Exception e){
            isInitialized = false;
        }

        this.signalTime = new ElapsedTime();
        sayInited();
    }
    private double voltageCompensation;
    private double targetVol;
    @Override
    protected void executeExt(Double... args) {
        double power = args[0];
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
        if (!isInitialized) sayBadInit();
        else {
            telemetry.addData("Power", getMotor().getPower());
        }
    }

    public static class InnerBuilder extends Builder<MotorWrapper> {
        @Override
        public InnerBuilder initialize(MainFile mainFile, String searchingDevice) {
            wrapper = new MotorWrapper(mainFile, searchingDevice);
            return this;
        }

        @Override
        public InnerBuilder setFields(Double... args) {
            wrapper.targetVol = args[0];
            wrapper.voltageCompensation = args[1];
            return this;
        }

        public InnerBuilder setDirection(DcMotorSimple.Direction direction){
            if(!wrapper.isInitialized) return this;
            wrapper.motor.setDirection(direction);
            return this;
        }
        public InnerBuilder setMode(DcMotor.RunMode runMode){
            if(!wrapper.isInitialized) return this;
            wrapper.motor.setMode(runMode);
            return this;
        }
        public InnerBuilder setBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior){
            if(!wrapper.isInitialized) return this;
            wrapper.motor.setZeroPowerBehavior(zeroPowerBehavior);
            return this;
        }
    }
    public static class InnerCollector extends CollectorBuilder<MotorWrapper>{

        @Override
        public InnerCollector add(MotorWrapper wrapper) {
            wrappers.put(wrapper.searchingDevice, wrapper);
            return this;
        }

        @Override
        public void showData() {
            for (MotorWrapper wrapper : wrappers.values()) {
                wrapper.showData();
            }
        }
    }
}
