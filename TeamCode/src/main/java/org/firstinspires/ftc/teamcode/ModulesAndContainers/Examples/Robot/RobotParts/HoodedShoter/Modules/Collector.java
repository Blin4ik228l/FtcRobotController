package org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.HoodedShoter.Modules;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.VoltageSensorClass;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.Wrappers.Examples.MotorWrapper;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Modules.Extenders.MotorModule;

public class Collector extends MotorModule {
    public String collector = controlHubDevices.getMotor(3);

    public Collector(OpMode op, VoltageSensorClass voltageSensorClass) {
        super(op);
        motorWrapper = motorBuilder.initialize(op, collector).setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER).setDirection(DcMotorSimple.Direction.FORWARD).setBehavior(DcMotor.ZeroPowerBehavior.FLOAT)
                .setFields(voltageSensorClass, 12.5, 1).get();

        sayInited();
    }

    public void setPower(double power){
        motorWrapper.setPower(power);
    }
    @Override
    public void showData() {

    }
}
