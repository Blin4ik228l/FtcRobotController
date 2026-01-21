package org.firstinspires.ftc.teamcode.Robot.RobotParts.DriveTrain;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Modules.Examples.Players.Pl2.AutoPlayerClass2;
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
        generalState = GeneralState.Get_pos;
    }
    private double foundedAngle;
    private double deltaAngle;
    private double range;
    public Position2D targetPos;
    public boolean flag;
    public AutoState autoState;
    public GeneralState generalState;
    public enum VyrState {
        Straight_to_it,
        Far_from_it
    }
    public enum GeneralState{
        Test,
        Get_pos,
        Execute_in_proccess,
        Stop
    }
    public enum AutoState{
        Check_readiness_for_start,
        Find_and_go_to_fire_pos,
        Find_and_go_to_artifacts
    }

    public CameraClass getCameraClass() {
        return cameraClass;
    }
    public OdometryClass getOdometryClass() {
        return odometryClass;
    }

    @Override
    public void resetTimer() {
        innerRunTime.reset();
        odometryClass.resetTimer();
        cameraClass.resetTimer();
    }

    @Override
    public void setIteration(int iterationCount) {
        super.setIteration(iterationCount);
        odometryClass.setIteration(iterationCount);
        cameraClass.setIteration(iterationCount);
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
        getNearestFirePos();

        switch (GeneralInformation.current.programName){
            case TeleOp:


                break;
            case Auto:
                switch (generalState){
                    case Test:
                        targetPos = new Position2D(odometryClass.getEncGlobalPosition2D().getX(), odometryClass.getEncGlobalPosition2D().getY(), toGlobalAngle(-45));
                        driveArgs = new Args.DriveArgs(targetPos.minus(odometryClass.getEncGlobalPosition2D()), 20);

                        generalState = GeneralState.Execute_in_proccess;
                        break;
                    case Get_pos:
                        switch (autoState){
                            case Check_readiness_for_start:
                                if(cameraClass.randomizeStatus == CameraClass.RandomizeStatus.UnDetected && cameraClass.onceSeen){
                                    switch (GeneralInformation.current.color){
                                        case Blue:
                                            switch (GeneralInformation.current.startPos){
                                                case Near_wall:
                                                    targetPos = new Position2D(odometryClass.getEncGlobalPosition2D().getX(), odometryClass.getEncGlobalPosition2D().getY(), toGlobalAngle(-45));
                                                    driveArgs = new Args.DriveArgs(targetPos.minus(odometryClass.getEncGlobalPosition2D()), 20);

                                                    autoState = AutoState.Find_and_go_to_fire_pos;
                                                    generalState = GeneralState.Execute_in_proccess;

                                                    minI = 2;
                                                    movementLogic = MovementLogic.Up_to_down;
                                                    break;
                                                case Far_from_wall:
                                                    targetPos = new Position2D(odometryClass.getEncGlobalPosition2D().getX(), odometryClass.getEncGlobalPosition2D().getY(), toGlobalAngle(0));
                                                    driveArgs = new Args.DriveArgs(targetPos.minus(odometryClass.getEncGlobalPosition2D()), 20);

                                                    autoState = AutoState.Find_and_go_to_fire_pos;
                                                    generalState = GeneralState.Execute_in_proccess;

                                                    minI = 8;
                                                    movementLogic = MovementLogic.Down_to_up;
                                                    break;
                                            }
                                            break;
                                        case Red:
                                            switch (GeneralInformation.current.startPos){
                                                case Near_wall:
                                                    targetPos = new Position2D(odometryClass.getEncGlobalPosition2D().getX(), odometryClass.getEncGlobalPosition2D().getY(), toGlobalAngle(45));
                                                    driveArgs = new Args.DriveArgs(targetPos.minus(odometryClass.getEncGlobalPosition2D()), 20);

                                                    autoState = AutoState.Find_and_go_to_fire_pos;
                                                    generalState = GeneralState.Execute_in_proccess;

                                                    minI = 2;
                                                    movementLogic = MovementLogic.Up_to_down;
                                                    break;
                                                case Far_from_wall:
                                                    targetPos = new Position2D(odometryClass.getEncGlobalPosition2D().getX(), odometryClass.getEncGlobalPosition2D().getY(), toGlobalAngle(0));
                                                    driveArgs = new Args.DriveArgs(targetPos.minus(odometryClass.getEncGlobalPosition2D()), 20);

                                                    autoState = AutoState.Find_and_go_to_fire_pos;
                                                    generalState = GeneralState.Execute_in_proccess;

                                                    minI = 8;
                                                    movementLogic = MovementLogic.Down_to_up;
                                                    break;
                                            }
                                            break;
                                    }
                                } else if (cameraClass.randomizeStatus == CameraClass.RandomizeStatus.Detected && !cameraClass.onceSeen) {

                                }else {

                                }
                                break;
                            case Find_and_go_to_fire_pos:
                                targetPos = getNearestFirePos();
                                driveArgs = new Args.DriveArgs(targetPos.minus(odometryClass.getEncGlobalPosition2D()), 20);

                                autoState = AutoState.Find_and_go_to_artifacts;
                                generalState = GeneralState.Execute_in_proccess;
                                break;

                            case Find_and_go_to_artifacts:
                                targetPos = getArtifactPos();
                                driveArgs = new Args.DriveArgs(targetPos.minus(odometryClass.getEncGlobalPosition2D()), 20);

                                generalState = GeneralState.Execute_in_proccess;
                                break;
                        }
                        break;
                    case Execute_in_proccess:
                        break;
                    case Stop:
                        break;
                }

                break;
        }
    }

    public double toGlobalAngle(double angle){
        return Math.toRadians(angle) - Math.toRadians(90);
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
    public void deleteArtifact(){
        GeneralInformation.current.generalObjects.getClosestArtifacts()[minI][0] = 0;
    }
    public enum MovementLogic{
        Up_to_down,
        Down_to_up
    }
    MovementLogic movementLogic;
    int minI;
    int mx = 2, mn = -1;
    public Position2D getArtifactPos() {
        Position2D artifactsPos = new Position2D();

        switch (movementLogic){
            case Down_to_up:
                if(minI == 0 && GeneralInformation.current.generalObjects.getClosestArtifacts()[minI][0] == 0){
                    break;
                }
                if(GeneralInformation.current.generalObjects.getClosestArtifacts()[minI][0] == 0){
                    minI --;
                    break;
                }

                break;
            case Up_to_down:
                if(GeneralInformation.current.generalObjects.getClosestArtifacts()[minI][0] == 0){
                    minI --;
                    minI = Range.clip(minI, mn, mx);
                }

                if(minI == -1){
                    minI = 5;
                    mn = 2;
                    mx = 5;
                } else if (minI == 2) {
                    minI = 8;
                    mn = 5;
                    mx = 8;
                }else if(minI == 5){
                    break;
                }
                break;
        }

        artifactsPos.setX(GeneralInformation.current.generalObjects.getClosestArtifacts()[minI][1]);
        artifactsPos.setY(GeneralInformation.current.generalObjects.getClosestArtifacts()[minI][2]);
        artifactsPos.setHeading(GeneralInformation.current.generalObjects.getClosestArtifacts()[minI][4]);

        return new Position2D(artifactsPos.getX(), artifactsPos.getY(), toGlobalAngle(artifactsPos.getHeading()));
    }
    public Position2D getNearestFirePos(){
        Position2D firePos;
        Vector2 nearDelta = new Vector2(GeneralInformation.current.generalObjects.getFireZones()[0][0] - odometryClass.getEncGlobalPosition2D().getX(), GeneralInformation.current.generalObjects.getFireZones()[0][1] - odometryClass.getEncGlobalPosition2D().getY());
        Vector2 farDelta = new Vector2(GeneralInformation.current.generalObjects.getFireZones()[1][0] - odometryClass.getEncGlobalPosition2D().getX(), GeneralInformation.current.generalObjects.getFireZones()[1][1] - odometryClass.getEncGlobalPosition2D().getY());

        //TODO Если наложиться угол
        if(nearDelta.length() > farDelta.length()){
            firePos = new Position2D(GeneralInformation.current.generalObjects.getFireZones()[1][0], GeneralInformation.current.generalObjects.getFireZones()[1][1], deltaAngle);
        }else firePos = new Position2D(GeneralInformation.current.generalObjects.getFireZones()[0][0], GeneralInformation.current.generalObjects.getFireZones()[0][1], deltaAngle);

        return firePos;
    }
    public Position2D getBasePos(){
        return new Position2D(GeneralInformation.current.generalObjects.getBaseCoord()[0], GeneralInformation.current.generalObjects.getBaseCoord()[1], Math.toRadians(90));
    }

    @Override
    public void showData() {
        telemetry.addLine("===POS CONTROLLER===");
        telemetry.addData("Founded Robot Angle", "%.1f°", foundedAngle * RAD);
        telemetry.addData("Target angle", "%.1f°", deltaAngle * RAD);
        telemetry.addData("Range to target", "%.1f cm", range);
        telemetry.addLine();

        cameraClass.showData();
        odometryClass.showData();
    }
}
