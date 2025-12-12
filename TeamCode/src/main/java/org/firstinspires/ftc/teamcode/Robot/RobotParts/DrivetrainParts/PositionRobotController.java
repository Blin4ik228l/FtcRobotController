package org.firstinspires.ftc.teamcode.Robot.RobotParts.DrivetrainParts;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Modules.Types.UpdatableModule;
import org.firstinspires.ftc.teamcode.Robot.RobotParts.DrivetrainParts.Odometry.ExOdometry;
import org.firstinspires.ftc.teamcode.Robot.RobotParts.DrivetrainParts.Odometry.Parts.MathUtils.Position2D;
import org.firstinspires.ftc.teamcode.Robot.RobotParts.DrivetrainParts.Odometry.Parts.MathUtils.Vector2;
import org.firstinspires.ftc.teamcode.Robot.TeamColor;

public class PositionRobotController extends UpdatableModule {
    public PositionRobotController(ExOdometry exOdometry, TeamColor teamColor, CameraClass cameraClass, OpMode op) {
        super(op.telemetry);
        this.exOdometry = exOdometry;
        this.cameraClass = cameraClass;
        this.teamColor = teamColor;
    }
    private TeamColor teamColor;
    private ExOdometry exOdometry;
    private CameraClass cameraClass;
    private Position2D rawPosition2D;

    private Position2D calcPos;        // Смешанная позиция
    private Position2D lastCameraPose;   // Последняя поза с камеры
    private double cameraWeight = 0; // Текущий вес камеры (0..1)

    // Настройки
    private static final double FADE_IN_RATE = 0.1;   // Скорость появления камеры
    private static final double FADE_OUT_RATE = 0.05; // Скорость исчезновения
    private static final double MAX_CAMERA_WEIGHT = 0.8; // Макс. доверие камере

    private Position2D filteredPose = null;
    private static final double ALPHA = 0.3;

    public double cameraFov = Math.toRadians(60);

    public boolean isVyrCompleted;

    @Override
    public void update() {
        exOdometry.update();

        if(exOdometry.robotCurVelocity.length() >= 15 || Math.abs(exOdometry.encHeadVel) >= Math.toRadians(5)){
            cameraClass.cameraLogic = CameraClass.CameraLogic.Check_condition;
            cameraClass.tagState = CameraClass.TagState.UnDetected;
        }else {
            cameraClass.update();
        }

        //Проверяем если робот двигается

        //Если камера увидела таг - обрабатываем
        if(cameraClass.tagState == CameraClass.TagState.Detected){
//            double targetWeight;
//
//            // Ограничиваем и умножаем на MAX_CAMERA_WEIGHT
//            targetWeight = Math.min(MAX_CAMERA_WEIGHT,
//                    Math.max(0.1, cameraClass.combinedWeight * MAX_CAMERA_WEIGHT));
//
//            if (cameraWeight < targetWeight) {
//                // Плавное появление камеры
//                cameraWeight += FADE_IN_RATE;
//                cameraWeight = Math.min(cameraWeight, targetWeight);
//            } else if (cameraWeight > targetWeight) {
//                // Плавное исчезновение камеры
//                cameraWeight -= FADE_OUT_RATE;
//                cameraWeight = Math.max(cameraWeight, targetWeight);
//            }
//
//            if (cameraWeight > 0.01 && cameraClass.tagState == CameraClass.TagState.UnDetected) {
//                double weight2 = 1.0 - cameraWeight;
//
//                calcPos = new Position2D(
//                        cameraClass.getLastRecordedPosition2D().getX() * cameraWeight + exOdometry.encGlobalPosition2D.getX() * weight2,
//                        cameraClass.getLastRecordedPosition2D().getY() * cameraWeight + exOdometry.encGlobalPosition2D.getY() * weight2,
//                        cameraClass.getLastRecordedPosition2D().getHeading() * cameraWeight + exOdometry.encGlobalPosition2D.getHeading() * weight2
//                );
//
//
//                lastCameraPose = cameraClass.getLastRecordedPosition2D();
//            } else {
//                calcPos = exOdometry.encGlobalPosition2D; // Только одометрия
//            }
//
//            if (filteredPose == null) {
//                filteredPose = calcPos;
//            }
//
//            filteredPose = new Position2D(
//                    ALPHA * calcPos.getX() + (1-ALPHA) * filteredPose.getX(),
//                    ALPHA * calcPos.getY() + (1-ALPHA) * filteredPose.getY(),
//                    ALPHA * filteredPose.getHeading() + (1 - ALPHA) * calcPos.getHeading()
//            );
            exOdometry.setPos(cameraClass.getLastRecordedPosition2D());
        }

    }
    public double getFoundedRobotAngle(){
        //TODO Изменить угол
        double targX = teamColor.getWallCoord()[0] - exOdometry.encGlobalPosition2D.getX();
        double targY = teamColor.getWallCoord()[1] - exOdometry.encGlobalPosition2D.getY();

        return new Position2D(0,0, Math.atan2(-targY, -targX)).getHeading();
    }

    public double getDeltaAngle(){
        double target = new Position2D(0,0,getFoundedRobotAngle() - exOdometry.encGlobalPosition2D.getHeading()).getHeading();

        if(Math.abs(target) < Math.toRadians(2)) {
            target = 0;
        }

        return target;
    }

    public double getRange(){
        return new Vector2(teamColor.getBlueWallCoord[0] - exOdometry.encGlobalPosition2D.getX(), teamColor.getBlueWallCoord[1] - exOdometry.encGlobalPosition2D.getY()).length();
    }
    public Position2D getDeltaPos(){
        Position2D targPos = new Position2D(teamColor.artifactsUnderBlueWallCoord[0][0], teamColor.artifactsUnderBlueWallCoord[0][1], teamColor.artifactsUnderBlueWallCoord[0][3]);

        return new Position2D(targPos.getX() - exOdometry.encGlobalPosition2D.getX(), targPos.getY() - exOdometry.encGlobalPosition2D.getY(), targPos.getHeading() - exOdometry.encGlobalPosition2D.getHeading());
    }

    public boolean isTagShouldBeInFov(){
        double leftBorder = exOdometry.encGlobalPosition2D.getHeading() + (Math.signum(exOdometry.encGlobalPosition2D.getHeading()) * cameraFov / 4.0);
        double rightBorder = exOdometry.encGlobalPosition2D.getHeading() - (Math.signum(exOdometry.encGlobalPosition2D.getHeading()) * cameraFov / 4.0);

        return !((Math.toRadians(45) < leftBorder && rightBorder < Math.toRadians(45))
                        ||
                (Math.toRadians(-45) > leftBorder && rightBorder > Math.toRadians(-45)));
    }
    public boolean isRobotHaveMinRange(){
        return getRange() < 200 && getRange() > 40;
    }
    @Override
    public void showData() {
        telemetry.addLine("===POS CONTROLLER===");
//        telemetry.addData("Position from class", "X:%.1f Y:%.1f H:%.1f°", filteredPose.getX(), filteredPose.getY(), filteredPose.getHeading() * 180/Math.PI);
        telemetry.addData("Founded Robot Angle", "%.1f°",getFoundedRobotAngle()* 180/Math.PI);
        telemetry.addData("Target angle", "%.1f°",getDeltaAngle()* 180/Math.PI);
        telemetry.addData("Range to target","%.1f cm", getRange());
        telemetry.addLine();
    }
}
