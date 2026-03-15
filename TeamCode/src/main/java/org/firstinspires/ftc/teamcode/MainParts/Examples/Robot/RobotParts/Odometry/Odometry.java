package org.firstinspires.ftc.teamcode.MainParts.Examples.Robot.RobotParts.Odometry;

import org.firstinspires.ftc.teamcode.MainParts.Examples.Robot.RobotParts.CameraClass;
import org.firstinspires.ftc.teamcode.MainParts.Examples.Robot.RobotParts.DriveTrain.DrivetrainParts.DrivetrainMotors;
import org.firstinspires.ftc.teamcode.MainParts.Examples.Robot.RobotParts.DriveTrain.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.MainParts.Examples.Robot.RobotParts.HoodedShoter.HoodedShooter;
import org.firstinspires.ftc.teamcode.MainParts.Examples.Robot.RobotParts.HoodedShoter.Modules.TurretMotor;
import org.firstinspires.ftc.teamcode.MainParts.Examples.Robot.RobotParts.DriveTrain.DrivetrainParts.GyroscopeClass;
import org.firstinspires.ftc.teamcode.MainParts.Modules.Extenders.UpdatableCollector;

public class Odometry extends UpdatableCollector {
    //Все энкодеры на телеге + гироскоп + камера  составляющие общую систему оценки положения робота в пространстве.
    public CameraClass cameraClass;
    private DrivetrainMotors.EncoderClass encoders;
    private GyroscopeClass gyro;
    private TurretMotor.EncodersClass encodersClass;
    private OdometryData outPutDataForRobot;
    public OdometryBuffer odometryBufferForRobot;
    private OdometryData outPutDataForTuret;
    public OdometryBuffer odometryBufferForTuret;

    public Odometry(MecanumDrivetrain drivetrain, HoodedShooter hoodedShooter, CameraClass cameraClass){
        super(false);
        outPutDataForRobot = new OdometryData();
        odometryBufferForRobot = new OdometryBuffer();

        outPutDataForTuret = new OdometryData();
        odometryBufferForTuret = new OdometryBuffer();

        this.cameraClass = cameraClass;

        encoders = drivetrain.motors.encoderClass;
        gyro = drivetrain.gyro;
        encodersClass = hoodedShooter.turretMotor.encodersClass;

        sayCreated();
    }
    public void setStartPos(OdometryData savedRobotData, OdometryData savedTurretData){
        outPutDataForTuret.setPosition(savedRobotData.getPosition());
        outPutDataForTuret.setPosition(savedTurretData.getPosition());
    }

    @Override
    protected void updateExt() {
        OdometryBuffer encodersBuf = encoders.encodersBuffer;
        OdometryBuffer gyroBuf = gyro.gyroBuffer;
        OdometryBuffer turretBuf = encodersClass.turretBuffer;

        outPutDataForRobot
                .setVelocity(encodersBuf.read().getVelocity())
                .setAccel(encodersBuf.read().getAccel())
                .setHeadVel(gyroBuf.read().getHeadVel())
                .setHeadAccel(gyroBuf.read().getHeadAccel());

        outPutDataForTuret
                .setHeadVel(turretBuf.read().getHeadVel() + outPutDataForRobot.getHeadVel())
                .setHeadAccel(turretBuf.read().getHeadVel() + outPutDataForRobot.getHeadAccel());

        //Камера видит таг -> Полностью считываем позицию с неё
        if (cameraClass.absoluteData.getDesisionMarg() > 0 && Math.abs(outPutDataForTuret.getHeadVel()) < MIN_TURRET_HEAD_SP){
            org.firstinspires.ftc.teamcode.MainParts.Examples.Robot.RobotParts.Odometry.Parts.MathUtils.Position2D pos = cameraClass.absoluteData.getPosition();

            outPutDataForRobot.getPosition().setX(pos.getX());
            outPutDataForRobot.getPosition().setY(pos.getY());
            outPutDataForRobot.getPosition().setHeading(pos.getHeading() - encodersClass.localHead);

            outPutDataForTuret.getPosition().setX(pos.getX());
            outPutDataForTuret.getPosition().setY(pos.getY());
            outPutDataForTuret.getPosition().setHeading(pos.getHeading());
        }else{
            org.firstinspires.ftc.teamcode.MainParts.Examples.Robot.RobotParts.Odometry.Parts.MathUtils.Position2D turretPos = turretBuf.read2().getPosition();
            org.firstinspires.ftc.teamcode.MainParts.Examples.Robot.RobotParts.Odometry.Parts.MathUtils.Position2D encodersPos = encodersBuf.read2().getPosition();
            org.firstinspires.ftc.teamcode.MainParts.Examples.Robot.RobotParts.Odometry.Parts.MathUtils.Position2D gyroPos = gyroBuf.read2().getPosition();

            //добавляем угол из буффера турели
            outPutDataForTuret.getPosition().add(0,0, turretPos.getHeading() + gyroPos.getHeading());

            //добавляем угол из буффера гироскопа
            outPutDataForRobot.getPosition().add(0, 0, gyroPos.getHeading());

            // Векторный поворот и добавление глобального перемещения к глобальным координатам
            org.firstinspires.ftc.teamcode.MainParts.Examples.Robot.RobotParts.Odometry.Parts.MathUtils.Vector2 deltaVector2 = new org.firstinspires.ftc.teamcode.MainParts.Examples.Robot.RobotParts.Odometry.Parts.MathUtils.Vector2(encodersPos.getX(), encodersPos.getY()).rotateToGlobal(outPutDataForRobot.getPosition().getHeading() + Math.toRadians(-90));

            outPutDataForRobot.getPosition().add(deltaVector2.x, deltaVector2.y , 0);
            outPutDataForTuret.getPosition().add(deltaVector2.x, deltaVector2.y , 0);
        }

//        outPutDataForRobot.rotateVelocity();
//        outPutDataForRobot.rotateAccel();

        odometryBufferForRobot.beginWrite().set(outPutDataForRobot);
        odometryBufferForRobot.endWrite();

        odometryBufferForTuret.beginWrite().set(outPutDataForTuret);
        odometryBufferForTuret.endWrite();
    }
    @Override
    protected void showDataExt() {
        telemetry.addLine("RobotData");
        telemetry.addData("Position", "X:%.1f Y:%.1f H:%.1f°", odometryBufferForRobot.read().getPosition().getX(), odometryBufferForRobot.read().getPosition().getY(), odometryBufferForRobot.read().getPosition().getHeading() * RAD);
        telemetry.addData("Velocity", "X:%.1fcm/s Y:%.1fcm/s, Len: %.2f", odometryBufferForRobot.read().getVelocity().x, odometryBufferForRobot.read().getVelocity().y, odometryBufferForRobot.read().getVelocity().length());
        telemetry.addData("Angular", "Vel:%.1f°/s Accel:%.1f°/s²", odometryBufferForRobot.read().getHeadVel() * RAD, odometryBufferForRobot.read().getHeadAccel() * RAD);
        telemetry.addLine("TuretData");
        telemetry.addData("Position", "X:%.1f Y:%.1f H:%.1f°", odometryBufferForTuret.read().getPosition().getX(), odometryBufferForTuret.read().getPosition().getY(), odometryBufferForTuret.read().getPosition().getHeading() * RAD);
        telemetry.addData("Angular", "Vel:%.1f°/s Accel:%.1f°/s²", odometryBufferForTuret.read().getHeadVel() * RAD, odometryBufferForTuret.read().getHeadAccel() * RAD);
        cameraClass.showData();
    }
}