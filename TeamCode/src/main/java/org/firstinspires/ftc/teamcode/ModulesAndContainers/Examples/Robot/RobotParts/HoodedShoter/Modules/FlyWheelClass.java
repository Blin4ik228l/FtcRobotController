package org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.HoodedShoter.Modules;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Modules.Extenders.MotorModule;

public class FlyWheelClass extends MotorModule {
    public FlyWheelClass(OpMode op) {
        super(op);
        motorsWrapper
                .add(op, dcMotorEx.initialize(op,"flyWheelRight").setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER).setBehavior(DcMotor.ZeroPowerBehavior.FLOAT))
                .add(op, dcMotorEx.initialize(op,"flyWheelLeft").setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER).setBehavior(DcMotor.ZeroPowerBehavior.FLOAT));
        sayInited();
    }
    public double curentSpeed;
    public double filteredVel;
    double alpha = 0.3;
    public double radius = 0.4;
    public boolean switcher = false;

    public double getCurVelocity(){
        if (!isInitialized) return 0;

        double left = motorsWrapper.get("motorLeft").getMotorEx().getVelocity(AngleUnit.RADIANS) * 19.2;
        double right = motorsWrapper.get("motorRight").getMotorEx().getVelocity(AngleUnit.RADIANS) * 19.2;

        double vel = left != 0 && right != 0 ? (left + right) / 2.0 : left + right;

        filteredVel = filteredVel * alpha + (1 - alpha) * vel;

        return switcher ? vel : vel * radius;
    }
    public void setPower(double power){
        if(!isInitialized) return;
        // 3. Компенсация напряжения
        double currentVoltage = voltageSensorClass.getCurVoltage();
        if (currentVoltage <= 0) currentVoltage = 11.5;

        double voltageMultiplier = 11.5 / currentVoltage;

        power *= voltageMultiplier;

        power = Range.clip(power, -1.0, 1.0);

        motorsWrapper.get("flyWheelRight").setPower(power);
        motorsWrapper.get("flyWheelLeft").setPower(-power);

        curentSpeed = getCurVelocity();
    }

    @Override
    public void showData() {
        motorsWrapper.showData();
    }
}
