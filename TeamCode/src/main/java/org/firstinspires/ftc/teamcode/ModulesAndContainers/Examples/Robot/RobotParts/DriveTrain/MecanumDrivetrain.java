package org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.DriveTrain;

import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.Config.MainFile;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.DriveTrain.DrivetrainParts.GyroscopeClass;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.DriveTrain.DrivetrainParts.DrivetrainMotors;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Modules.Extenders.MainModule;

public class MecanumDrivetrain extends MainModule {
    //Телега робота(моторы + колёса) с энкодерами и гироскопом .
    public DrivetrainMotors motors;
    public GyroscopeClass gyro;
    public MecanumDrivetrain(MainFile mainFile){
        super(mainFile);

        gyro = new GyroscopeClass(mainFile, controlHubDevices.gyroskope);
        motors = new DrivetrainMotors(mainFile);

        sayCreated();
    }


    @Override
    protected void showDataExt() {
        motors.showData();
    }
}
