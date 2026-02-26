package org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.DriveTrain;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.DriveTrain.DrivetrainParts.GyroscopeClass;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.VoltageSensorClass;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Modules.Extenders.UpdatableModule;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.DriveTrain.DrivetrainParts.DrivetrainMotors;

public class MecanumDrivetrain extends UpdatableModule {
    //Телега робота(моторы + колёса) с энкодерами и гироскопом .
    public DrivetrainMotors motors;
    public GyroscopeClass gyro;
    public MecanumDrivetrain(OpMode op, VoltageSensorClass voltageSensorClass){
        super(op);

        gyro = new GyroscopeClass(op);
        motors = new DrivetrainMotors(op, voltageSensorClass);

        this.isInitialized = motors.isInitialized && gyro.isInitialized;
        sayInited();
    }
    @Override
    public void update() {
        gyro.update();
    }

    @Override
    public void showData() {
        sayModuleName();
        motors.showData();
        telemetry.addLine();
    }
}
