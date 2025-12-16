package org.firstinspires.ftc.teamcode.Robot.RobotParts.DrivetrainParts;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Modules.Types.UpdatableModule;
import org.firstinspires.ftc.teamcode.Robot.RobotParts.DrivetrainParts.Odometry.OdometryClass;
import org.firstinspires.ftc.teamcode.Robot.RobotParts.DrivetrainParts.Odometry.Parts.MathUtils.Position2D;
import org.firstinspires.ftc.teamcode.Robot.RobotParts.DrivetrainParts.Odometry.Parts.MathUtils.Vector2;

public class PositionRobotController extends UpdatableModule {
    public PositionRobotController(OdometryClass odometryClass, TeamColorClass teamColorClass, CameraClass cameraClass, OpMode op) {
        super(op.telemetry);
        this.odometryClass = odometryClass;
        this.cameraClass = cameraClass;
        this.teamColorClass = teamColorClass;
    }

    private TeamColorClass teamColorClass;
    private OdometryClass odometryClass;
    private CameraClass cameraClass;
    private double kPower;
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

    private double cameraFov = Math.toRadians(60);
    private double foundedAngle;
    private double deltaAngle;
    private double range;
    private Position2D firePos;

    public enum VyrState {
        Straight_to_it,
        Near_from_it,
        Far_from_it
    }

    public VyrState vyrState = VyrState.Far_from_it;

    public void setKPower(double kPower){
        this.kPower = kPower;
    }

    @Override
    public void update() {
        odometryClass.update();

        if (odometryClass.robotCurVelocity.length() >= 15 || Math.abs(odometryClass.encHeadVel) >= Math.toRadians(5)) {
            cameraClass.cameraLogic = CameraClass.CameraLogic.Check_condition;
            cameraClass.tagState = CameraClass.TagState.UnDetected;
        } else {
            cameraClass.update();
        }

        //Проверяем если робот двигается

        //Если камера увидела таг - обрабатываем
        if (cameraClass.tagState == CameraClass.TagState.Detected) {
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
            odometryClass.setPos(cameraClass.getLastRecordedPosition2D());
        }

        calcRange();
        calcAngle();
        calcDeltaAngle();
        calcNearestFirePoses();
    }

    public double getDeltaAngle() {
        return deltaAngle;
    }

    public double getRange() {
        return range;
    }
    public void calcRange() {
        range = new Vector2(teamColorClass.getTagCoord()[0] - odometryClass.encGlobalPosition2D.getX(), teamColorClass.getTagCoord()[1] - odometryClass.encGlobalPosition2D.getY()).length();
    }
    public void calcAngle(){
        //TODO Изменить угол
        double targX = teamColorClass.getPointVyr()[0] - odometryClass.encGlobalPosition2D.getX();
        double targY = teamColorClass.getPointVyr()[1] - odometryClass.encGlobalPosition2D.getY();

        foundedAngle = new Position2D(0, 0, Math.atan2(-targY, -targX)).getHeading();
    }

    public void calcDeltaAngle() {
        deltaAngle = new Position2D(0, 0, foundedAngle - odometryClass.encGlobalPosition2D.getHeading()).getHeading();

        if (Math.abs(deltaAngle) < Math.toRadians(2)) {
            deltaAngle = 0;
            vyrState = VyrState.Straight_to_it;
        } else if (Math.abs(deltaAngle) < Math.toRadians(9)) {
            vyrState = VyrState.Near_from_it;
        } else vyrState = VyrState.Far_from_it;
    }

    public Position2D getDeltaPos() {
        Position2D targPos = new Position2D(teamColorClass.getClosestArtifacts()[0][0], teamColorClass.getClosestArtifacts()[0][1], teamColorClass.getClosestArtifacts()[0][3]);

        return new Position2D(targPos.getX() - odometryClass.encGlobalPosition2D.getX(), targPos.getY() - odometryClass.encGlobalPosition2D.getY(), targPos.getHeading() - odometryClass.encGlobalPosition2D.getHeading());
    }

    public boolean isTagShouldBeInFov() {
        double leftBorder = odometryClass.encGlobalPosition2D.getHeading() + (Math.signum(odometryClass.encGlobalPosition2D.getHeading()) * cameraFov / 4.0);
        double rightBorder = odometryClass.encGlobalPosition2D.getHeading() - (Math.signum(odometryClass.encGlobalPosition2D.getHeading()) * cameraFov / 4.0);

        return !((Math.toRadians(45) < leftBorder && rightBorder < Math.toRadians(45))
                ||
                (Math.toRadians(-45) > leftBorder && rightBorder > Math.toRadians(-45)));
    }

    public boolean isRobotHaveMinRange() {
        return range < 200 && range > 40;
    }
    public void calcNearestFirePoses(){
        Vector2 nearDelta = new Vector2(teamColorClass.getFireZones()[0][0] - odometryClass.encGlobalPosition2D.getX(), teamColorClass.getFireZones()[0][1] - odometryClass.encGlobalPosition2D.getY());
        Vector2 farDelta = new Vector2(teamColorClass.getFireZones()[1][0] - odometryClass.encGlobalPosition2D.getX(), teamColorClass.getFireZones()[1][1] - odometryClass.encGlobalPosition2D.getY());

        //Смотрим хватит ли напряжения на выстрел с дальней позиции
        if(kPower > 0.75){
            if(nearDelta.length() > farDelta.length()){
                firePos = new Position2D(teamColorClass.getFireZones()[1][0], teamColorClass.getFireZones()[1][1], deltaAngle);
            }else firePos = new Position2D(teamColorClass.getFireZones()[0][0], teamColorClass.getFireZones()[0][1], deltaAngle);

        }else firePos = new Position2D(teamColorClass.getFireZones()[0][0], teamColorClass.getFireZones()[0][1], deltaAngle);

    }
    public Position2D getBasePos(){
        return new Position2D(teamColorClass.getBaseCoord()[0], teamColorClass.getBaseCoord()[1], Math.toRadians(90));
    }

    public Position2D getFirePos() {
        return firePos;
    }

    @Override
    public void showData() {
        telemetry.addLine("===POS CONTROLLER===");
//        telemetry.addData("Position from class", "X:%.1f Y:%.1f H:%.1f°", filteredPose.getX(), filteredPose.getY(), filteredPose.getHeading() * 180/Math.PI);
        telemetry.addData("Founded Robot Angle", "%.1f°", foundedAngle * RAD);
        telemetry.addData("Target angle", "%.1f°", deltaAngle * RAD);
        telemetry.addData("Range to target", "%.1f cm", range);
        telemetry.addLine();
    }
}
