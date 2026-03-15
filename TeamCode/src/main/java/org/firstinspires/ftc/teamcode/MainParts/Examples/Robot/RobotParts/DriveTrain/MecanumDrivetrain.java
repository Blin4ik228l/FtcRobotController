package org.firstinspires.ftc.teamcode.MainParts.Examples.Robot.RobotParts.DriveTrain;

import org.firstinspires.ftc.teamcode.MainParts.Examples.Robot.RobotParts.DriveTrain.DrivetrainParts.GyroscopeClass;
import org.firstinspires.ftc.teamcode.MainParts.Examples.Robot.RobotParts.DriveTrain.DrivetrainParts.DrivetrainMotors;
import org.firstinspires.ftc.teamcode.MainParts.Modules.Extenders.MainModule;

public class MecanumDrivetrain extends MainModule {
    //Телега робота(моторы + колёса) с энкодерами и гироскопом .
    public DrivetrainMotors motors;
    public GyroscopeClass gyro;
    public MecanumDrivetrain(){
        gyro = new GyroscopeClass();
        motors = new DrivetrainMotors();

        sayCreated();
    }


    @Override
    protected void showDataExt() {
        motors.showData();
    }
}
