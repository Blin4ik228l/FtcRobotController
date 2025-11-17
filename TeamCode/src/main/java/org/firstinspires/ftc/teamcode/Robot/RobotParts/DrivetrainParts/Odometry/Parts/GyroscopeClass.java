package org.firstinspires.ftc.teamcode.Robot.RobotParts.DrivetrainParts.Odometry.Parts;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.Modules.Module;

import java.util.concurrent.TimeUnit;

public class GyroscopeClass extends Module {
    public GyroscopeClass(OpMode op){
        super(op.telemetry);
        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = op.hardwareMap.get(IMU.class, "imu");
        parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
        ));

        imu.initialize(parameters);

        imu.resetYaw();

        telemetry.addLine("Gyroscope Inited");
    }
    public Deadline imuResetTime = new Deadline(500, TimeUnit.MILLISECONDS);
    public IMU.Parameters parameters;
    public IMU imu;

    public void showData(){
        telemetry.addLine("===GYRO===");
        telemetry.addData("Yaw", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        telemetry.addLine();
    }
}