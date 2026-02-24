package org.firstinspires.ftc.teamcode.ModulesAndContainers.Modules.Extenders;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.HardwareBuilder;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.Wrappers.MotorWrapper;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Modules.Module;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.VoltageSensorClass;

public abstract class MotorModule extends Module {
    protected VoltageSensorClass voltageSensorClass;
    protected MotorWrapper.Builder motorsWrapper;
    protected DcMotorBuilder dcMotor;
    protected DcMotorExBuilder dcMotorEx;
    public MotorModule(OpMode op) {
        super(op);

        motorsWrapper = new MotorWrapper.Builder();
        dcMotor = new DcMotorBuilder();
        dcMotorEx = new DcMotorExBuilder();
    }
    public void setVoltageSensorClass(VoltageSensorClass voltageSensorClass) {
        this.voltageSensorClass = voltageSensorClass;
    }

    public class DcMotorBuilder extends MotorBuilder<DcMotorBuilder>{
        public DcMotor dcMotor;

        @Override
        public DcMotorBuilder initialize(OpMode op, String deviceName) {
            this.deviceName = deviceName;
            try {
                dcMotor = hardwareMap.get(DcMotor.class, deviceName);
            }catch (Exception e){
                isInit = false;
            }
            isInitialized = isInit;
            return this;
        }
        @Override
        public DcMotorBuilder setDirection(DcMotorSimple.Direction direction){
            if(!isInitialized) return this;
            dcMotor.setDirection(direction);
            return this;
        }
        @Override
        public DcMotorBuilder setMode(DcMotor.RunMode runMode){
            if(!isInitialized) return this;
            dcMotor.setMode(runMode);
            return this;
        }
        @Override
        public DcMotorBuilder setBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior){
            if(!isInitialized) return this;
            dcMotor.setZeroPowerBehavior(zeroPowerBehavior);
            return this;
        }

        @Override
        public DcMotor get(){
            return dcMotor;
        }


    }
    public class DcMotorExBuilder extends MotorBuilder<DcMotorExBuilder>{
        public DcMotorEx dcMotorEx;

        @Override
        public DcMotorExBuilder initialize(OpMode op, String deviceName) {
            this.deviceName = deviceName;
            try {
                dcMotorEx = hardwareMap.get(DcMotorEx.class, deviceName);
            }catch (Exception e){
                isInit = false;
            }
            isInitialized = isInit;
            return this;
        }
        @Override
        public DcMotorExBuilder setDirection(DcMotorSimple.Direction direction){
            if(!isInitialized) return this;
            dcMotorEx.setDirection(direction);
            return this;
        }
        @Override
        public DcMotorExBuilder setMode(DcMotor.RunMode runMode){
            if(!isInitialized) return this;
            dcMotorEx.setMode(runMode);
            return this;
        }
        @Override
        public DcMotorExBuilder setBehavior(DcMotorEx.ZeroPowerBehavior zeroPowerBehavior){
            if(!isInitialized) return this;
            dcMotorEx.setZeroPowerBehavior(zeroPowerBehavior);
            return this;
        }

        @Override
        public DcMotorEx get(){
            return dcMotorEx;
        }


    }
    public abstract class MotorBuilder <T> extends HardwareBuilder {
        public boolean isInit = true;
        public String deviceName;
        public abstract T setDirection(DcMotorSimple.Direction direction);
        public abstract T setMode(DcMotor.RunMode runMode);
        public abstract T setBehavior(DcMotorEx.ZeroPowerBehavior zeroPowerBehavior);
        public abstract DcMotorSimple get();
    }
}
