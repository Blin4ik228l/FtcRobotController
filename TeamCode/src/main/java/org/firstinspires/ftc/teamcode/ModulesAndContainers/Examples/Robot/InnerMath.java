package org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Players.PL0.Units;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.Config.MainFile;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.Wrappers.Examples.MotorWrapper;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Modules.Extenders.Extenders2.UpdatableModule;

public class InnerMath{
    private double COUNTS_PER_ENCODER_REV = 1;
    private double DRIVE_GEAR_REDUCTION = 1;
    private double radius = 1;
    private double COUNTS_PER_CM = 1;

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
    public InnerMath calculateCountsPerCm(){
        COUNTS_PER_CM = (COUNTS_PER_ENCODER_REV * DRIVE_GEAR_REDUCTION)/
                (radius * 2 * Math.PI);
        return this;
    }
    private double ticksToCm(double ticks){
        return ticks / COUNTS_PER_CM;
    }
    public double getCurentPos(MotorWrapper motorWrapper, Units units){
        DcMotor motor;

        if(motorWrapper != null)motor = motorWrapper.getMotor();
        else motor = null;


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
    public double getCurrentVelocity(MotorWrapper motorWrapper, Units units){
        DcMotorEx motorEx;
        if(motorWrapper != null) motorEx = motorWrapper.getMotorEx();
        else motorEx = null;

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