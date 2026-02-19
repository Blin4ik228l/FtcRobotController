package org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.HoodedShoter.Modules;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.Wrappers.MotorWrapper;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Modules.Extenders.MotorModule;

public class Collector extends MotorModule {
    public Collector(OpMode op) {
        super(op);
        motorsWrapper.add(op, dcMotorEx.initialize( "inTake").setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER).setBehavior(DcMotor.ZeroPowerBehavior.FLOAT).get());
    }
    public void setPower(double power){
        motorsWrapper.get("inTake").setPower(power);
    }

    public MotorWrapper getMotor(){
        return motorsWrapper.get("inTake");
    }
    @Override
    public void showData() {

    }
}
