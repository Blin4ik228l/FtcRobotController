package org.firstinspires.ftc.teamcode.ModulesAndContainers.Modules.Extenders;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Players.PL0.Units;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.Wrappers.Examples.MotorWrapper;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Modules.Module;

import java.util.HashMap;

public abstract class MotorModule extends Module {
    protected Builder motorsWrapper;
    protected MotorWrapper.Builder motorBuilder;
    protected MotorWrapper motorWrapper;
    protected InnerMath innerMath;
    public MotorModule(OpMode op) {
        super(op);

        motorsWrapper = new Builder();
        motorBuilder = new MotorWrapper.Builder();
        innerMath = new InnerMath();
    }



    public class Builder{
        private HashMap<String, MotorWrapper> motors = new HashMap<>();

        public Builder add(MotorWrapper motorWrapper) {
            motors.put(motorWrapper.getSearchingDevice(), motorWrapper);
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

    public class InnerMath{
        private double COUNTS_PER_ENCODER_REV = 1;
        private double DRIVE_GEAR_REDUCTION = 1;
        private double radius = 1;
        private double COUNTS_PER_CM = (COUNTS_PER_ENCODER_REV * DRIVE_GEAR_REDUCTION)/
                (radius * 2 * Math.PI);

        public InnerMath setCOUNTS_PER_ENCODER_REV(double COUNTS_PER_ENCODER_REV) {
            this.COUNTS_PER_ENCODER_REV = COUNTS_PER_ENCODER_REV;
            return this;
        }

        public InnerMath setDRIVE_GEAR_REDUCTION(double DRIVE_GEAR_REDUCTION) {
            this.DRIVE_GEAR_REDUCTION = DRIVE_GEAR_REDUCTION;
            return this;
        }

        public InnerMath setRadius(double radius) {
            this.radius = radius;
            return this;
        }
        private double ticksToCm(double ticks){
            return ticks / COUNTS_PER_CM;
        }
        public double getCurentPos(String motorName, Units units){
            DcMotor motor;
            if(motorWrapper != null) motor = motorWrapper.getMotor();
            else motor = motorsWrapper.get(motorName).getMotor();

            if(motor == null) return 0;
            else {
                double pos = -motor.getCurrentPosition();
                switch (units){
                    case Cm:
                        pos = ticksToCm(pos);
                        break;
                    case Rad:
                        pos = ticksToCm(pos) / radius;
                        break;
                    default:
                        break;
                }
                return pos;
            }
        }
        public double getCurrentVelocity(String motorName, Units units){
            DcMotorEx motorEx;
            if(motorWrapper != null) motorEx = motorWrapper.getMotorEx();
            else motorEx = motorsWrapper.get(motorName).getMotorEx();


            if(motorEx == null) return 0;
            else {
                double vel = -motorEx.getVelocity();
                switch (units){
                    case Cm:
                        vel = ticksToCm(vel);
                        break;
                    case Rad:
                        vel = ticksToCm(vel) / radius;
                        break;
                    default:
                        break;
                }

                return vel;
            }
        }
    }

}
