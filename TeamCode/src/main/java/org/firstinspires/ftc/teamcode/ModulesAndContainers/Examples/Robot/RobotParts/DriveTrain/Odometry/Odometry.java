package org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.DriveTrain.Odometry;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.CameraClass;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.DriveTrain.DrivetrainParts.DrivetrainMotors;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.DriveTrain.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.HoodedShoter.HoodedShoter;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.HoodedShoter.Modules.TurretMotor;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Modules.Extenders.UpdatableModule;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.DriveTrain.Odometry.Parts.GyroscopeClass;

public class Odometry extends UpdatableModule{
    //Все энкодеры на телеге + гироскоп + камера  составляющие общую систему оценки положения робота в пространстве.
    public CameraClass cameraClass;
    private DrivetrainMotors.EncoderClass encoders;
    private TurretMotor.TurretOdometry turretOdometry;
    private GyroscopeClass gyro;

    private OdometryData outPutData;

    public OdometryBuffer odometryBuffer;

    public Odometry(OpMode op, MecanumDrivetrain drivetrain, HoodedShoter hoodedShoter){
        super(op);

        cameraClass = hoodedShoter.cameraClass;
        encoders = drivetrain.motors.encoderClass;
        turretOdometry = hoodedShoter.turretMotor.turretOdometry;

        telemetry.addLine("ExOdometry Inited");
    }

    @Override
    public void update(){
        cameraClass.update();
        gyro.update();

        outPutData
                .setRobotVelocity(encoders.getRawData().getRobotVelocity())
                .setRobotAccel(encoders.getRawData().getRobotAccel())
                .setRobotHeadVel(encoders.getRawData().getRobotHeadVel())
                .setRobotHeadAccel(encoders.getRawData().getRobotHeadAccel());

        //Камера видит таг -> Полностью считываем позицию с неё
        if (cameraClass.absoluteData.getDesisionMarg() > 0){
            outPutData.setRobotPosition(cameraClass.absoluteData.getRobotPosition());
        }else{
            outPutData.getRobotPosition().add(0, 0, encoders.getRawData().getRobotPosition().getHeading());

            // Векторный поворот и добавление глобального перемещения к глобальным координатам
            outPutData.setRotatableVector(encoders.getRawData().getRobotPosition().getX(), encoders.getRawData().getRobotPosition().getY());
            outPutData.getRotatableVector().rotateToGlobal(outPutData.getRobotPosition().getHeading());

            outPutData.getRobotPosition().add( outPutData.getRotatableVector().x,  outPutData.getRotatableVector().y , 0);
        }

        outPutData.rotateVelocity();
        outPutData.rotateAccel();

        odometryBuffer.beginWrite().set(outPutData);
        odometryBuffer.endWrite();
    }

    @Override
    public void showData(){
        telemetry.addLine("===ODOMETRY===");
        if(isInizialized) {
            telemetry.addData("Position enc", "X:%.1f Y:%.1f H:%.1f°", odometryBuffer.read().getRobotPosition().getX(), odometryBuffer.read().getRobotPosition().getY(), odometryBuffer.read().getRobotPosition().getHeading() * 180 / Math.PI);
            telemetry.addData("Velocity", "X:%.1fcm/s Y:%.1fcm/s, Len: %.2f", odometryBuffer.read().getRobotVelocity().x, odometryBuffer.read().getRobotVelocity().y, odometryBuffer.read().getRobotVelocity().length());
            telemetry.addData("Angular", "Vel:%.1f°/s Accel:%.1f°/s²", odometryBuffer.read().getRobotHeadVel() * 180 / Math.PI, odometryBuffer.read().getRobotHeadAccel() * 180 / Math.PI);
        }
        telemetry.addLine();

        if (gyro.isInizialized) gyro.showData();
        if (encoders.isInizialized) encoders.showData();
        if (cameraClass.isInizialized) cameraClass.showData();
    }


}