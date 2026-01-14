package org.firstinspires.ftc.teamcode.Robot.RobotParts.DriveTrain.DrivetrainParts;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Modules.Types.UpdatableModule;
import org.firstinspires.ftc.teamcode.Robot.RobotParts.DriveTrain.DrivetrainParts.Odometry.OdometryClass;
import org.firstinspires.ftc.teamcode.Robot.RobotParts.DriveTrain.DrivetrainParts.Odometry.Parts.MathUtils.Position2D;
import org.firstinspires.ftc.teamcode.Robot.RobotParts.DriveTrain.DrivetrainParts.Odometry.Parts.MathUtils.Vector2;

public class PositionRobotController extends UpdatableModule {
    private TeamClass teamClass;
    private OdometryClass odometryClass;
    private CameraClass cameraClass;
    public PositionRobotController(OdometryClass odometryClass, TeamClass teamClass, CameraClass cameraClass, OpMode op) {
        super(op.telemetry);
        this.odometryClass = odometryClass;
        this.cameraClass = cameraClass;
        this.teamClass = teamClass;
    }
    private double kPower;
    private double cameraFov = Math.toRadians(60);
    private double foundedAngle;
    private double deltaAngle;
    private double range;
    private Position2D firePos;

    public enum VyrState {
        Straight_to_it,
        Far_from_it
    }


    public VyrState vyrState = VyrState.Far_from_it;

    public void setKPower(double kPower){
        this.kPower = kPower;
    }

    @Override
    public void update() {
        odometryClass.updateSpeed();
//        if(odometryClass.moveState != OdometryClass.MoveState.Stopped && odometryClass.rotateState != OdometryClass.RotateState.Stopped)
//        {cameraClass.reset();}
        cameraClass.update();
        if(odometryClass.getStopTime().seconds() < 1){
            if (cameraClass.tagState == CameraClass.TagState.Detected) {
                odometryClass.setPos(cameraClass.getLastRecordedPosition2D());
//                cameraClass.tagState = CameraClass.TagState.UnDetected;
            }else {
                odometryClass.updatePoses();
            }
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
        range = new Vector2(teamClass.getTagCoord()[0] - odometryClass.getEncGlobalPosition2D().getX(), teamClass.getTagCoord()[1] - odometryClass.getEncGlobalPosition2D().getY()).length();
    }
    public void calcAngle(){
        double targX = teamClass.getPointVyr()[0] - odometryClass.getEncGlobalPosition2D().getX();
        double targY = teamClass.getPointVyr()[1] - odometryClass.getEncGlobalPosition2D().getY();
//        -targY, -targX
        foundedAngle = new Position2D(0, 0, Math.atan2(targX, -targY)).getHeading();
    }

    public void calcDeltaAngle() {
        /*Если позиция с камеры была хоть раз получена, значит робот выравниться в правильном направлении*/
        deltaAngle = new Position2D(0, 0, foundedAngle - odometryClass.getEncGlobalPosition2D().getHeading()).getHeading();

        if (Math.abs(deltaAngle) < Math.toRadians(2)) {
            deltaAngle = 0;

            if(range >= 150 && cameraClass.tagState == CameraClass.TagState.Detected) vyrState = VyrState.Straight_to_it;
            else if (range < 150 && cameraClass.onceSeen) vyrState = VyrState.Straight_to_it;

        } else vyrState = VyrState.Far_from_it;
    }

    public Position2D getDeltaPos() {
        Position2D targPos = new Position2D(teamClass.getClosestArtifacts()[0][0], teamClass.getClosestArtifacts()[0][1], teamClass.getClosestArtifacts()[0][3]);

        return new Position2D(targPos.getX() - odometryClass.getEncGlobalPosition2D().getX(), targPos.getY() - odometryClass.getEncGlobalPosition2D().getY(), targPos.getHeading() - odometryClass.getEncGlobalPosition2D().getHeading());
    }

    public boolean isTagShouldBeInFov() {
        double leftBorder = odometryClass.getEncGlobalPosition2D().getHeading() + (Math.signum(odometryClass.getEncGlobalPosition2D().getHeading()) * cameraFov / 4.0);
        double rightBorder = odometryClass.getEncGlobalPosition2D().getHeading() - (Math.signum(odometryClass.getEncGlobalPosition2D().getHeading()) * cameraFov / 4.0);

        return !((Math.toRadians(45) < leftBorder && rightBorder < Math.toRadians(45))
                ||
                (Math.toRadians(-45) > leftBorder && rightBorder > Math.toRadians(-45)));
    }

    public boolean isRobotHaveMinRange() {
        return range < 200 && range > 40;
    }
    public void calcNearestFirePoses(){
        Vector2 nearDelta = new Vector2(teamClass.getFireZones()[0][0] - odometryClass.getEncGlobalPosition2D().getX(), teamClass.getFireZones()[0][1] - odometryClass.getEncGlobalPosition2D().getY());
        Vector2 farDelta = new Vector2(teamClass.getFireZones()[1][0] - odometryClass.getEncGlobalPosition2D().getX(), teamClass.getFireZones()[1][1] - odometryClass.getEncGlobalPosition2D().getY());

        //Смотрим хватит ли напряжения на выстрел с дальней позиции
        if(kPower > 0.75){
            if(nearDelta.length() > farDelta.length()){
                firePos = new Position2D(teamClass.getFireZones()[1][0], teamClass.getFireZones()[1][1], deltaAngle);
            }else firePos = new Position2D(teamClass.getFireZones()[0][0], teamClass.getFireZones()[0][1], deltaAngle);

        }else firePos = new Position2D(teamClass.getFireZones()[0][0], teamClass.getFireZones()[0][1], deltaAngle);

    }
    public Position2D getBasePos(){
        return new Position2D(teamClass.getBaseCoord()[0], teamClass.getBaseCoord()[1], Math.toRadians(90));
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
