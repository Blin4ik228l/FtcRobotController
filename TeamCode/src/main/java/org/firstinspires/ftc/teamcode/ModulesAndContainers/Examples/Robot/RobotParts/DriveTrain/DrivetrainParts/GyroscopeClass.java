package org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.DriveTrain.DrivetrainParts;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.Config.MainFile;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.Odometry.OdometryBuffer;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.Odometry.OdometryData;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.Odometry.Parts.MathUtils.Position2D;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Modules.Extenders.Extenders2.UpdatableModule;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Modules.Extenders.UpdatingModule;

import java.util.concurrent.TimeUnit;

public class GyroscopeClass extends UpdatableModule {
    public IMU imu;
    public IMU.Parameters parameters;
    public GyroscopeClass(MainFile mainFile, String searchingDevice){
        super(mainFile, searchingDevice);
        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        try {
            imu = hardwareMap.get(IMU.class, searchingDevice);
            parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.UP,
                    RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
            ));
            imu.initialize(parameters);

            imu.resetYaw();
        } catch (Exception e) {
            isInitialized = false;
        }

        selfMath = new SelfMath();

        gyroBuffer = new OdometryBuffer();
        sayInited();
    }
    public OdometryBuffer gyroBuffer;
    public Deadline imuResetTime = new Deadline(500, TimeUnit.MILLISECONDS);
    public SelfMath selfMath;
    @Override
    protected void updateExt() {
        selfMath.calculateAll();
    }
    @Override
    public void showDataExt() {
        telemetry.addData("Yaw", gyroBuffer.read().getPosition().getHeading());
        telemetry.addLine();
    }

    private class SelfMath{
        private OdometryData rawData;
        private double gyroCurHeading, gyroDeltaHeading, gyroLastHeading;
        private double gyroCurHeadVel, gyroDeltaHeadVel, gyroLastHeadVel;
        private ElapsedTime runTime ;//Время с начала запуска программы
        private double currentTime;//В разных точках пограммы будет обозначать время когда брались значения с датчиков
        private double deltaTimes;
        private double oldTimes;// Предыдущее время
        private double fltrdHeadVel;
        public SelfMath(){
            rawData = new OdometryData();
            runTime = new ElapsedTime();
        }
        public void calculateAll(){
            updateGyroHeading(AngleUnit.RADIANS);
            updateGyroHeadVel();
            updateDeltaGyroHeadVel();
            updateGyroHeadAccel();

            gyroBuffer.beginWrite().set(rawData);
            gyroBuffer.endWrite2();
        }
        private void updateGyroHeading(AngleUnit angleUnit) {
            gyroCurHeading = imu.getRobotYawPitchRollAngles().getYaw(angleUnit);
            gyroDeltaHeading = gyroCurHeading - gyroLastHeading;
            gyroLastHeading = gyroCurHeading;

            rawData.setPosition(new Position2D(0, 0, gyroDeltaHeading));
        }
        private void updateGyroHeadVel(){
            double filtr = 0.2;
            fltrdHeadVel = filtr * imu.getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate + (1 - filtr) * fltrdHeadVel;

            gyroCurHeadVel = fltrdHeadVel;

            currentTime = runTime.milliseconds();// Время в которое мы фиксируем показания с датчиков

            rawData.setHeadVel(gyroCurHeadVel);
        }
        private void updateDeltaGyroHeadVel(){
            gyroDeltaHeadVel = gyroCurHeadVel - gyroLastHeadVel;
            gyroLastHeadVel = gyroCurHeadVel;

            deltaTimes = (currentTime - oldTimes) / 1000;
            oldTimes = currentTime;
        }
        private void updateGyroHeadAccel(){
            double gyroHeadAccel;

            if(deltaTimes == 0) {gyroHeadAccel = 0;}
            else gyroHeadAccel = gyroDeltaHeadVel / deltaTimes;

            rawData.setHeadAccel(gyroHeadAccel);
        }
    }
}