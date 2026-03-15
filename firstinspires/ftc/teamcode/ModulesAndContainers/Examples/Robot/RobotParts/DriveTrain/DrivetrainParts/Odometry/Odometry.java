package org.firstinspires.ftc.teamcode.MainParts.Examples.Robot.RobotParts.DriveTrain.DrivetrainParts.Odometry;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MainParts.Examples.Robot.RobotParts.HoodedShoter.Modules.TurretClass;
import org.firstinspires.ftc.teamcode.MainParts.Modules.Extenders.UpdatableModule;
import org.firstinspires.ftc.teamcode.MainParts.Examples.Robot.RobotParts.DriveTrain.DrivetrainParts.Odometry.Parts.EncoderClass;
import org.firstinspires.ftc.teamcode.MainParts.Examples.Robot.RobotParts.DriveTrain.DrivetrainParts.Odometry.Parts.GyroscopeClass;

public class Odometry extends UpdatableModule implements Runnable{
    //Все энкодеры на телеге + гироскоп + камера  составляющие общую систему оценки положения робота в пространстве.
    public Thread thread;

    private GyroscopeClass gyro;
    private EncoderClass encoders;
    private TurretClass turret;

    private OdometryData outPutData;
    public OdometryBuffer odometryBuffer;

    public Odometry(OpMode op){
        super(op);
        telemetry = op.telemetry;

        try {
            gyro = new GyroscopeClass(op);
        } catch (Exception e) {
            gyro.isInizialized = false;
        }
        try {
            encoders = new EncoderClass(op);
        } catch (Exception e) {
            encoders.isInizialized = false;
        }
        try {
            turret = new TurretClass(op);
        } catch (Exception e) {
            turret.isInizialized = false;
        }

        this.thread = new Thread(this);
        this.thread.start();

        telemetry.addLine("ExOdometry Inited");
    }
    @Override
    public void run() {
        while (!Thread.currentThread().isInterrupted())
        {
            long start = System.nanoTime();
            update();

            long elapsed = System.nanoTime() - start;
            long sleepTime = 20_000_000 - elapsed; // 20ms в наносекундах

            if (sleepTime > 0) {
                try {
                    Thread.sleep(20);
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                }
            }
        }
    }
    @Override
    public void update(){
        if(!(encoders.isInizialized || gyro.isInizialized || turret.isInizialized)) {isInizialized = false; return;}

        turret.update();
        encoders.update();
        gyro.update();

        outPutData
                .setRobotVelocity(encoders.getRawData().getRobotVelocity())
                .setRobotAccel(encoders.getRawData().getRobotAccel())
                .setRobotHeadVel(encoders.getRawData().getRobotHeadVel())
                .setRobotHeadAccel(encoders.getRawData().getRobotHeadAccel());

        //Камера видит таг -> Полностью считываем позицию с неё
        if (turret.dataO.getDesisionMarg() > 0){
            outPutData.setRobotPosition(turret.dataO.getRobotPosition());
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
        if (turret.isInizialized) turret.showData();
    }


}