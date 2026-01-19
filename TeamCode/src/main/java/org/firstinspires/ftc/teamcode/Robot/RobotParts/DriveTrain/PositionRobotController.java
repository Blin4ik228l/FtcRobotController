package org.firstinspires.ftc.teamcode.Robot.RobotParts.DriveTrain;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Modules.Types.UpdatableModule;
import org.firstinspires.ftc.teamcode.Robot.GeneralInformation;
import org.firstinspires.ftc.teamcode.Robot.RobotParts.DriveTrain.DrivetrainParts.CameraClass;
import org.firstinspires.ftc.teamcode.Robot.RobotParts.DriveTrain.DrivetrainParts.Odometry.OdometryClass;
import org.firstinspires.ftc.teamcode.Robot.RobotParts.DriveTrain.DrivetrainParts.Odometry.Parts.MathUtils.Position2D;
import org.firstinspires.ftc.teamcode.Robot.RobotParts.DriveTrain.DrivetrainParts.Odometry.Parts.MathUtils.Vector2;
import org.firstinspires.ftc.teamcode.TaskAndArgs.Args;

public class PositionRobotController extends UpdatableModule {
    private final OdometryClass odometryClass;
    private final CameraClass cameraClass;
    public PositionRobotController(OpMode op) {
        super(op);

        this.odometryClass = new OdometryClass(op);
        this.cameraClass = new CameraClass(op);

        vyrState = VyrState.Far_from_it;
        autoState = AutoState.Check_readiness_for_start;
    }
    private double foundedAngle;
    private double deltaAngle;
    private double range;
    private Position2D firePos;
    public Position2D targetPos;
    public boolean flag;
    public enum VyrState {
        Straight_to_it,
        Far_from_it
    }
    public AutoState autoState;
    public boolean allowMove;
    public enum AutoState{
        Check_readiness_for_start,
        Find_and_go_to_fire_pos,
        Find_and_go_to_artifacts
    }

    public CameraClass getCameraClass() {
        return cameraClass;
    }

    @Override
    public void resetTimer() {
        innerRunTime.reset();
        odometryClass.resetTimer();
        cameraClass.resetTimer();
    }

    public OdometryClass getOdometryClass() {
        return odometryClass;
    }

    public VyrState vyrState;
    public Args.DriveArgs driveArgs;

    @Override
    public void update() {
        odometryClass.updateSpeed();
        cameraClass.update();

        if (odometryClass.moveState == OdometryClass.MoveState.Stopped && odometryClass.rotateState == OdometryClass.RotateState.Stopped)
        {
            if (cameraClass.tagState == CameraClass.TagState.Detected && !flag) {
                odometryClass.setPos(cameraClass.getLastRecordedPosition2D());
                flag = true;
            }
        }else {
            if (cameraClass.onceSeen)
            {
                odometryClass.updatePoses();
                flag = false;
            }
        }

        calcRange();
        calcAngle();
        calcDeltaAngle();
        calcNearestFirePoses();

        switch (GeneralInformation.current.programName){
            case TeleOp:
                break;
            case Auto:
                switch (autoState){
                    case Find_and_go_to_fire_pos:
                        if(cameraClass.randomizeStatus == CameraClass.RandomizeStatus.UnDetected && cameraClass.onceSeen){
                            if(GeneralInformation.current.color == GeneralInformation.Color.Blue && GeneralInformation.current.startPos == GeneralInformation.StartPos.Near_wall){
                                targetPos = new Position2D(firePos.getX(), firePos.getY(), Math.toRadians(45 - 90));
                                driveArgs = new Args.DriveArgs(targetPos.minus(odometryClass.getEncGlobalPosition2D()), 20);
                                allowMove = true;
                            }
                        }
                        break;
                }
                break;
        }
    }

    public double getDeltaAngle() {
        return deltaAngle;
    }

    public double getRange() {
        return range;
    }
    public void calcRange() {
        range = new Vector2(GeneralInformation.current.generalObjects.getTagCoord()[0] - odometryClass.getEncGlobalPosition2D().getX(), GeneralInformation.current.generalObjects.getTagCoord()[1] - odometryClass.getEncGlobalPosition2D().getY()).length();
    }
    public void calcAngle(){
        double targX = GeneralInformation.current.generalObjects.getPointVyr()[0] - odometryClass.getEncGlobalPosition2D().getX();
        double targY = GeneralInformation.current.generalObjects.getPointVyr()[1] - odometryClass.getEncGlobalPosition2D().getY();

        foundedAngle = new Position2D(0, 0, Math.atan2(targX, -targY)).getHeading();
    }

    public void calcDeltaAngle() {
        /*Если позиция с камеры была хоть раз получена, значит робот выравниться в правильном направлении*/
        deltaAngle = new Position2D(0, 0, foundedAngle - odometryClass.getEncGlobalPosition2D().getHeading()).getHeading();

        if (Math.abs(deltaAngle) < Math.toRadians(2)) {
            deltaAngle = 0;

            if(range >= 150 && cameraClass.tagState == CameraClass.TagState.Detected) vyrState = PositionRobotController.VyrState.Straight_to_it;
            else if (range < 150 && cameraClass.onceSeen) vyrState = PositionRobotController.VyrState.Straight_to_it;

        } else vyrState = PositionRobotController.VyrState.Far_from_it;
    }

    public Position2D getDeltaPos() {
        Position2D targPos = new Position2D(GeneralInformation.current.generalObjects.getClosestArtifacts()[0][0], GeneralInformation.current.generalObjects.getClosestArtifacts()[0][1], GeneralInformation.current.generalObjects.getClosestArtifacts()[0][3]);

        return new Position2D(targPos.getX() - odometryClass.getEncGlobalPosition2D().getX(), targPos.getY() - odometryClass.getEncGlobalPosition2D().getY(), targPos.getHeading() - odometryClass.getEncGlobalPosition2D().getHeading());
    }
    public void calcNearestFirePoses(){
        Vector2 nearDelta = new Vector2(GeneralInformation.current.generalObjects.getFireZones()[0][0] - odometryClass.getEncGlobalPosition2D().getX(), GeneralInformation.current.generalObjects.getFireZones()[0][1] - odometryClass.getEncGlobalPosition2D().getY());
        Vector2 farDelta = new Vector2(GeneralInformation.current.generalObjects.getFireZones()[1][0] - odometryClass.getEncGlobalPosition2D().getX(), GeneralInformation.current.generalObjects.getFireZones()[1][1] - odometryClass.getEncGlobalPosition2D().getY());

        //Смотрим хватит ли напряжения на выстрел с дальней позиции
        if(nearDelta.length() > farDelta.length()){
            firePos = new Position2D(GeneralInformation.current.generalObjects.getFireZones()[1][0], GeneralInformation.current.generalObjects.getFireZones()[1][1], deltaAngle);
        }else firePos = new Position2D(GeneralInformation.current.generalObjects.getFireZones()[0][0], GeneralInformation.current.generalObjects.getFireZones()[0][1], deltaAngle);
    }
    public Position2D getBasePos(){
        return new Position2D(GeneralInformation.current.generalObjects.getBaseCoord()[0], GeneralInformation.current.generalObjects.getBaseCoord()[1], Math.toRadians(90));
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
