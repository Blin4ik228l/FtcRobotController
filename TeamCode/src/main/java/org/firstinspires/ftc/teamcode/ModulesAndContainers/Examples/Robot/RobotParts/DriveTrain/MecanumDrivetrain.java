package org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.DriveTrain;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.Config.MainFile;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.DriveTrain.DrivetrainParts.GyroscopeClass;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.VoltageSensorClass;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Modules.Extenders.Extenders2.UpdatableModule;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.DriveTrain.DrivetrainParts.DrivetrainMotors;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Modules.Extenders.UpdatingModule;

public class MecanumDrivetrain extends UpdatingModule {
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
    protected void updateExt() {
        gyro.update();
    }

    @Override
    protected void showDataExt() {
        motors.showData();
        telemetry.addLine();
    }
}
