package org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.DriveTrain;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.CameraClass;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.DriveTrain.Odometry.Odometry;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.DriveTrain.Odometry.OdometryData;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.VoltageSensorClass;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Modules.Extenders.UpdatableModule;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.DriveTrain.DrivetrainParts.DrivetrainMotors;

public class MecanumDrivetrain extends UpdatableModule {
    //Телега робота(моторы + колёса) с энкодерами, гироскопом и камерой.
    public DrivetrainMotors motors;
    public MecanumDrivetrain(OpMode op, VoltageSensorClass voltageSensorClass){
        super(op);
        try {
            motors = new DrivetrainMotors(op);
        }catch (Exception e){
            motors.isInizialized = false;
        }

        motors.setVoltageSensorClass(voltageSensorClass);

        telemetry.addLine("Drivetrain Inited");
    }
    public void setVoltageSensor(VoltageSensorClass voltageSensor){
        motors.setVoltageSensorClass(voltageSensor);
    }

    @Override
    public void update() {

    }

    @Override
    public void showData() {
        telemetry.addLine("===DRIVETRAIN DATA===");
        motors.showData();
    }
}
