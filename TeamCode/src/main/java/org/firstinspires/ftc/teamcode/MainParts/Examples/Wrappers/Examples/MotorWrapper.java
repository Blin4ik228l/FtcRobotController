package org.firstinspires.ftc.teamcode.MainParts.Examples.Wrappers.Examples;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MainParts.Examples.Players.Enums.Units;
import org.firstinspires.ftc.teamcode.MainParts.Examples.Robot.Config.MainFile;
import org.firstinspires.ftc.teamcode.MainParts.Examples.Robot.InnerMath;
import org.firstinspires.ftc.teamcode.MainParts.Examples.Robot.RobotParts.VoltageSensorClass;
import org.firstinspires.ftc.teamcode.MainParts.Modules.Extenders.Extenders2.ExecutableModule;

public class MotorWrapper extends ExecutableModule {
    private DcMotor motor;
    private MotorConfigurationType motorConfigurationType;
    private static VoltageSensorClass voltageSensorClass;
    private InnerMath innerMath = new InnerMath();
    private ElapsedTime signalTime;

    public MotorWrapper(String searchingDevice) {
        super(searchingDevice);

        try {
            motor = hardwareMap.get(DcMotor.class, searchingDevice);

            if (motor instanceof DcMotorEx){
                motor = (DcMotorEx) motor;
            }
            voltageSensorClass = MainFile.voltageSensorClass;
            motorConfigurationType = motor.getMotorType();
        }catch (Exception e){
            isInitialized = false;
        }

        this.signalTime = new ElapsedTime();
        sayInited();
    }
    private double voltageCompensation;
    private double targetVol;
    public double getPower(){
        return motor.getPower();
    }

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

    public double getCurPos(Units units){
        return innerMath.getCurentPos(this, units);
    }
    public double getCurVel(Units units){
        return innerMath.getCurrentVelocity(this, units);
    }
    @Override
    protected void showDataExt() {
        if (!isInitialized) sayBadInit();
        else {
            telemetry.addData("Power", getMotor().getPower());
        }
    }
    public static class InnerBuilder extends Builder<MotorWrapper> {
        @Override
        public InnerBuilder initialize(String searchingDevice) {
            wrapper = new MotorWrapper(searchingDevice);
            return this;
        }

        @Override
        public InnerBuilder setFields(Double... args) {
            wrapper.targetVol = args[0];
            wrapper.voltageCompensation = args[1];
            if(args[2] == 0){
                if(wrapper.isInitialized){
                    args[2] = wrapper.motorConfigurationType.getTicksPerRev();
                }
            }
            wrapper.innerMath.setCOUNTS_PER_ENCODER_REV(args[2]).setRadius(args[3]).setDRIVE_GEAR_REDUCTION(args[4]).calculateCountsPerCm();
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
