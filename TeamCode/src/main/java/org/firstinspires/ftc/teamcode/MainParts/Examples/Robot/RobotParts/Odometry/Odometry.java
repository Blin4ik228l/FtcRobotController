package org.firstinspires.ftc.teamcode.MainParts.Examples.Robot.RobotParts.Odometry;

import org.firstinspires.ftc.teamcode.MainParts.Examples.Robot.RobotParts.CameraClass;
import org.firstinspires.ftc.teamcode.MainParts.Examples.Robot.RobotParts.DriveTrain.DrivetrainParts.DrivetrainMotors;
import org.firstinspires.ftc.teamcode.MainParts.Examples.Robot.RobotParts.DriveTrain.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.MainParts.Examples.Robot.RobotParts.HoodedShoter.HoodedShooter;
import org.firstinspires.ftc.teamcode.MainParts.Examples.Robot.RobotParts.HoodedShoter.Modules.TurretMotor;
import org.firstinspires.ftc.teamcode.MainParts.Examples.Robot.RobotParts.DriveTrain.DrivetrainParts.GyroscopeClass;
import org.firstinspires.ftc.teamcode.MainParts.Examples.Robot.RobotParts.Odometry.Parts.MathUtils.Position2D;
import org.firstinspires.ftc.teamcode.MainParts.Examples.Robot.RobotParts.Odometry.Parts.MathUtils.Vector2;
import org.firstinspires.ftc.teamcode.MainParts.Modules.Extenders.UpdatableCollector;

public class Odometry extends UpdatableCollector {
    //Все энкодеры на телеге + гироскоп + камера  составляющие общую систему оценки положения робота в пространстве.
    public final CameraClass cameraClass;
    private final DrivetrainMotors motors;
    private final GyroscopeClass gyro;
    private final TurretMotor turretMotor;
    private final OdometryData dataForRobot;
    private final OdometryData dataForTurret;
    public final OdometryBuffer bufferForRobot;
    public final OdometryBuffer bufferForTurret;

    public Odometry(MecanumDrivetrain drivetrain, HoodedShooter hoodedShooter, CameraClass cameraClass){
        super(false);
        dataForRobot = new OdometryData();
        bufferForRobot = new OdometryBuffer();

        dataForTurret = new OdometryData();
        bufferForTurret = new OdometryBuffer();

        this.cameraClass = cameraClass;

        motors = drivetrain.motors;
        gyro = drivetrain.gyro;
        turretMotor = hoodedShooter.turretMotor;

        sayCreated();
    }
    public void setStartPos(OdometryData savedRobotData, OdometryData savedTurretData){
        dataForTurret.setPosition(savedRobotData.getPosition());
        dataForTurret.setPosition(savedTurretData.getPosition());
    }

    @Override
    protected void updateExt() {
        OdometryBuffer encodersBuf = motors.encodersBuffer;
        OdometryBuffer gyroBuf = gyro.gyroBuffer;
        OdometryBuffer turretBuf = turretMotor.turretBuffer;

        dataForRobot
                .setVelocity(encodersBuf.read().getVelocity())
                .setAccel(encodersBuf.read().getAccel())
                .setHeadVel(gyroBuf.read().getHeadVel())
                .setHeadAccel(gyroBuf.read().getHeadAccel());

        dataForTurret
                .setHeadVel(turretBuf.read().getHeadVel() + dataForRobot.getHeadVel())
                .setHeadAccel(turretBuf.read().getHeadVel() + dataForRobot.getHeadAccel());

        //Камера видит таг -> Полностью считываем позицию с неё
        if (cameraClass.absoluteData.getDesisionMarg() > 0 && Math.abs(dataForTurret.getHeadVel()) < MIN_TURRET_HEAD_SP){
            Position2D pos = cameraClass.absoluteData.getPosition();

            dataForRobot.getPosition().setX(pos.getX());
            dataForRobot.getPosition().setY(pos.getY());
            dataForRobot.getPosition().setHeading(pos.getHeading() - turretMotor.localHead);

            dataForTurret.getPosition().setX(pos.getX());
            dataForTurret.getPosition().setY(pos.getY());
            dataForTurret.getPosition().setHeading(pos.getHeading());

            turretBuf.read2().getPosition();
            encodersBuf.read2().getPosition();
            gyroBuf.read2().getPosition();
        }else{
            Position2D turretPos = turretBuf.read2().getPosition();
            Position2D encodersPos = encodersBuf.read2().getPosition();
            Position2D gyroPos = gyroBuf.read2().getPosition();

            //добавляем угол из буффера турели
            dataForTurret.getPosition().add(0,0, turretPos.getHeading() + gyroPos.getHeading());

            //добавляем угол из буффера гироскопа
            dataForRobot.getPosition().add(0, 0, gyroPos.getHeading());

            // Векторный поворот и добавление глобального перемещения к глобальным координатам
            Vector2 deltaVector2 = new Vector2(encodersPos.getX(), encodersPos.getY()).rotateToGlobal(dataForRobot.getPosition().getHeading() + Math.toRadians(-90));

            dataForRobot.getPosition().add(deltaVector2.x, deltaVector2.y , 0);
            dataForTurret.getPosition().add(deltaVector2.x, deltaVector2.y , 0);
        }

//        outPutDataForRobot.rotateVelocity();
//        outPutDataForRobot.rotateAccel();

        bufferForRobot.beginWrite().set(dataForRobot);
        bufferForRobot.endWrite();

        bufferForTurret.beginWrite().set(dataForTurret);
        bufferForTurret.endWrite();
    }
    @Override
    protected void showDataExt() {
        telemetry.addLine("RobotData");
        telemetry.addData("Position", "X:%.1f Y:%.1f H:%.1f°", bufferForRobot.read().getPosition().getX(), bufferForRobot.read().getPosition().getY(), bufferForRobot.read().getPosition().getHeading() * RAD);
        telemetry.addData("Velocity", "X:%.1fcm/s Y:%.1fcm/s, Len: %.2f", bufferForRobot.read().getVelocity().x, bufferForRobot.read().getVelocity().y, bufferForRobot.read().getVelocity().length());
        telemetry.addData("Angular", "Vel:%.1f°/s Accel:%.1f°/s²", bufferForRobot.read().getHeadVel() * RAD, bufferForRobot.read().getHeadAccel() * RAD);
        telemetry.addLine("TuretData");
        telemetry.addData("Position", "X:%.1f Y:%.1f H:%.1f°", bufferForTurret.read().getPosition().getX(), bufferForTurret.read().getPosition().getY(), bufferForTurret.read().getPosition().getHeading() * RAD);
        telemetry.addData("Angular", "Vel:%.1f°/s Accel:%.1f°/s²", bufferForTurret.read().getHeadVel() * RAD, bufferForTurret.read().getHeadAccel() * RAD);
        cameraClass.showData();
    }
}