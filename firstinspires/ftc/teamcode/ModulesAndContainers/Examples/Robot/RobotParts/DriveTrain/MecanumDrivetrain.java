package org.firstinspires.ftc.teamcode.MainParts.Examples.Robot.RobotParts.DriveTrain;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.MainParts.Examples.Robot.RobotParts.DriveTrain.DrivetrainParts.NavigationSystem;
import org.firstinspires.ftc.teamcode.MainParts.Modules.Extenders.UpdatableModule;
import org.firstinspires.ftc.teamcode.MainParts.Examples.Robot.RobotParts.DriveTrain.DrivetrainParts.DrivetrainMotors;

public class MecanumDrivetrain extends UpdatableModule {
    //Телега робота(моторы + колёса) с энкодерами, гироскопом и камерой.
    public DrivetrainMotors motors;
    public NavigationSystem navigationSystem;
    public MecanumDrivetrain(OpMode op){
        super(op);
        try {
            motors = new DrivetrainMotors(op);
        }catch (Exception e){
            motors.isInizialized = false;
        }

        navigationSystem = new NavigationSystem(op);

        telemetry.addLine("Drivetrain Inited");
    }

    @Override
    public void update() {
        navigationSystem.update();
    }

    @Override
    public void showData() {
        telemetry.addLine("===DRIVETRAIN DATA===");
        motors.showData();
        navigationSystem.showData();
        motors.showData();
    }
}
