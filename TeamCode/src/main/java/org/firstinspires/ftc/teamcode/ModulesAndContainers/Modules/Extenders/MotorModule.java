package org.firstinspires.ftc.teamcode.ModulesAndContainers.Modules.Extenders;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

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

    public class DcMotorBuilder{
        public DcMotor dcMotor;
        public DcMotorBuilder initialize(String deviceName) {
            dcMotor = hardwareMap.get(DcMotor.class, deviceName);
            return this;
        }
        public DcMotorBuilder setDirection(DcMotorSimple.Direction direction){
            dcMotor.setDirection(direction);
            return this;
        }
        public DcMotorBuilder setMode(DcMotor.RunMode runMode){
            dcMotor.setMode(runMode);
            return this;
        }
        public DcMotorBuilder setBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior){
            dcMotor.setZeroPowerBehavior(zeroPowerBehavior);
            return this;
        }
        public DcMotor get(){
            return dcMotor;
        }
    }
    public class DcMotorExBuilder{
        public DcMotorEx dcMotorEx;

        public DcMotorExBuilder initialize(String deviceName) {
            dcMotorEx = hardwareMap.get(DcMotorEx.class, deviceName);
            return this;
        }
        public DcMotorExBuilder setDirection(DcMotorSimple.Direction direction){
            dcMotorEx.setDirection(direction);
            return this;
        }
        public DcMotorExBuilder setMode(DcMotor.RunMode runMode){
            dcMotorEx.setMode(runMode);
            return this;
        }
        public DcMotorExBuilder setBehavior(DcMotorEx.ZeroPowerBehavior zeroPowerBehavior){
            dcMotorEx.setZeroPowerBehavior(zeroPowerBehavior);
            return this;
        }
        public DcMotorEx get(){
            return dcMotorEx;
        }
    }
}
