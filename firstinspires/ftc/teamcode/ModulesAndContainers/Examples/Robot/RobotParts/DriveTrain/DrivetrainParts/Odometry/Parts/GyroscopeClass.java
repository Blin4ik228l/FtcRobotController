package org.firstinspires.ftc.teamcode.MainParts.Examples.Robot.RobotParts.DriveTrain.DrivetrainParts.Odometry.Parts;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.MainParts.Modules.Extenders.UpdatableModule;
import org.firstinspires.ftc.teamcode.MainParts.Examples.Robot.RobotParts.DriveTrain.DrivetrainParts.Odometry.OdometryData;
import org.firstinspires.ftc.teamcode.MainParts.Examples.Robot.RobotParts.DriveTrain.DrivetrainParts.Odometry.Parts.MathUtils.Position2D;

import java.util.concurrent.TimeUnit;

public class GyroscopeClass extends UpdatableModule {
    public GyroscopeClass(OpMode op){
        super(op);
        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        try {
            imu = hardwareMap.get(IMU.class, "imu");
        } catch (Exception e) {
            isInizialized = false;
            return;
        }


        parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
        ));
        imu.initialize(parameters);

        imu.resetYaw();

        rawData = new OdometryData();
        selfMath = new SelfMath();

        telemetry.addLine("Gyroscope Inited");
    }
    private OdometryData rawData;
    public Deadline imuResetTime = new Deadline(500, TimeUnit.MILLISECONDS);
    public IMU.Parameters parameters;
    public IMU imu;
    public SelfMath selfMath;

    @Override
    public void update() {
        if (!isInizialized) return;
        selfMath.calculateAll();
    }

    @Override
    public void showData(){
        telemetry.addLine("===GYRO===");

        if (isInizialized) {
            telemetry.addData("Yaw", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        }else{
            telemetry.addLine("DEVICE NOT FOUND");
        }

        telemetry.addLine();
    }
    private class SelfMath{
        private double gyroCurHeading, gyroDeltaHeading, gyroLastHeading;
        private double gyroCurHeadVel, gyroDeltaHeadVel, gyroLastHeadVel;
        private ElapsedTime runTime ;//Время с начала запуска программы
        private double currentTime;//В разных точках пограммы будет обозначать время когда брались значения с датчиков
        private final double []deltaTimes;
        private final double [] oldTimes;// Предыдущее время

        public OdometryData getRawData() {
            return rawData;
        }

        public SelfMath(){
            oldTimes = new double[1];                                         // Предыдущее время
            deltaTimes = new double[1];
            runTime = new ElapsedTime();
        }
        public void calculateAll(){
            updateGyroHeading(AngleUnit.RADIANS);
            updateGyroHeadVel();
            updateDeltaGyroHeadVel();
            updateGyroHeadAccel();
        }
        private void updateGyroHeading(AngleUnit angleUnit) {
            gyroCurHeading = imu.getRobotYawPitchRollAngles().getYaw(angleUnit);
            gyroDeltaHeading = gyroCurHeading - gyroLastHeading;
            gyroLastHeading = gyroCurHeading;

            rawData.setRobotPosition(new Position2D(0, 0, gyroDeltaHeading));
        }
        private void updateGyroHeadVel(){
            gyroCurHeadVel = imu.getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate;

            currentTime = runTime.milliseconds();// Время в которое мы фиксируем показания с датчиков

            rawData.setRobotHeadVel(gyroCurHeadVel);
        }
        private void updateDeltaGyroHeadVel(){
            gyroDeltaHeadVel = gyroCurHeadVel - gyroLastHeadVel;
            gyroLastHeadVel = gyroCurHeadVel;

            deltaTimes[0] = (currentTime - oldTimes[0]) / 1000;
            oldTimes[0] = currentTime;
        }
        private void updateGyroHeadAccel(){
            double gyroHeadAccel;

            if(deltaTimes[0] == 0) {gyroHeadAccel = 0;}
            else gyroHeadAccel = gyroDeltaHeadVel / deltaTimes[0];

            rawData.setRobotHeadAccel(gyroHeadAccel);
        }
    }
}