package org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.HoodedShoter;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.HoodedShoter.Modules.AngleController;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.HoodedShoter.Modules.Collector;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.HoodedShoter.Modules.FlyWheelClass;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.HoodedShoter.Modules.TurretMotor;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.VoltageSensorClass;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.Wrappers.Examples.MotorWrapper;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Modules.Extenders.UpdatableModule;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.HoodedShoter.Modules.DigitalCellsClass;

public class HoodedShoter extends UpdatableModule {
    public AngleController angleController;
    public TurretMotor turretMotor;
    public FlyWheelClass flyWheelClass;
    public MotorWrapper collector;
    public DigitalCellsClass digitalCellsClass;

    public HoodedShoter(OpMode op, VoltageSensorClass voltageSensorClass) {
        super(op);
        turretMotor = new TurretMotor(op, new MotorWrapper.Builder().initialize(op, controlHubDevices.getMotor(1)).setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER).setDirection(DcMotorSimple.Direction.FORWARD).setBehavior(DcMotor.ZeroPowerBehavior.FLOAT)
                .setFields(voltageSensorClass, 12.5, 1).get());
        flyWheelClass = new FlyWheelClass(op, voltageSensorClass);

        collector = new MotorWrapper.Builder().initialize(op, controlHubDevices.getMotor(1)).setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER).setDirection(DcMotorSimple.Direction.FORWARD).setBehavior(DcMotor.ZeroPowerBehavior.FLOAT)
                .setFields(voltageSensorClass, 12.5, 1).get();

        angleController = new AngleController(op, "servo1");

        digitalCellsClass = new DigitalCellsClass(op);

        this.isInitialized = turretMotor.isInitialized && flyWheelClass.isInitialized && collector.isInitialized && angleController.isInitialized && digitalCellsClass.isInitialized;
        sayInited();
    }

    @Override
    public void update() {
        digitalCellsClass.update();
    }

    @Override
    public void showData() {
        telemetry.addLine("===COLLECTOR DATA===");
//        digitalCellsClass.showData();
        turretMotor.showData();
//        collector.showData();
//        flyWheelClass.showData();
        telemetry.addLine();
    }
}
