package org.firstinspires.ftc.teamcode.MainParts.Examples.Robot.RobotParts.DriveTrain;

import org.firstinspires.ftc.teamcode.MainParts.Examples.Robot.RobotParts.DriveTrain.DrivetrainParts.GyroscopeClass;
import org.firstinspires.ftc.teamcode.MainParts.Examples.Robot.RobotParts.DriveTrain.DrivetrainParts.DrivetrainMotors;
import org.firstinspires.ftc.teamcode.MainParts.Modules.Extenders.MainModule;
import org.firstinspires.ftc.teamcode.MainParts.Modules.Extenders.UpdatableCollector;

public class MecanumDrivetrain extends UpdatableCollector {
    //Телега робота(моторы + колёса) с энкодерами и гироскопом .
    public DrivetrainMotors motors;
    public GyroscopeClass gyro;
    public MecanumDrivetrain(){
        super(true);
        gyro = new GyroscopeClass();
        motors = new DrivetrainMotors();

        sayCreated();
    }

    @Override
    protected void updateExt() {
        motors.encoderClass.update(iterationCount, 1);
        gyro.update(iterationCount, 1);
    }

    @Override
    protected void showDataExt() {
        motors.showData();
        gyro.showData();
    }
}
