package org.firstinspires.ftc.teamcode.Robot.RobotParts.DriveTrain;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Modules.Types.UpdatableModule;
import org.firstinspires.ftc.teamcode.Robot.GeneralInformation;
import org.firstinspires.ftc.teamcode.Robot.RobotParts.DriveTrain.DrivetrainParts.CameraClass;
import org.firstinspires.ftc.teamcode.Robot.RobotParts.DriveTrain.DrivetrainParts.Odometry.OdometryClass;
import org.firstinspires.ftc.teamcode.Robot.RobotParts.DriveTrain.DrivetrainParts.Odometry.Parts.MathUtils.Position2D;
import org.firstinspires.ftc.teamcode.Robot.RobotParts.DriveTrain.DrivetrainParts.Odometry.Parts.MathUtils.Vector2;
import org.firstinspires.ftc.teamcode.TaskAndArgs.Args;

public class PositionRobotController extends UpdatableModule {
    private OdometryClass odometryClass;
    private final CameraClass cameraClass;
    public PositionRobotController(OpMode op) {
        super(op);

//        this.odometryClass = new OdometryClass(op);
        this.cameraClass = new CameraClass(op);

        vyrState = VyrState.Far_from_it;

        switch (GeneralInformation.current.programName){
            case TeleOp:
                generalState = GeneralState.STOP;
                break;
            case Auto:
                generalState = GeneralState.Get_pos;
                autoState = AutoState.CHECK_READINESS_FOR_START;
                break;
        }

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
        EXECUTE_IN_PROCCESS,
        DONE,
        DELETE_ARTIFACT,
        STOP,
        EMERGENCY_RATTLING

    }
    public enum AutoState{
        CHECK_READINESS_FOR_START,
        FIND_AND_GO_TO_FIRE_POS,
        GO_TO_LOAD_POS,
        GO_AFORE_ARTIFACTS,
        FIND_AND_GO_TO_ARTIFACTS
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
    public Args.DriveArgs[] pathDriveArgs = new Args.DriveArgs[2];

    public MovementLogic movementLogic;
    public int minI;
    public int mx = 2, mn = -1;
    public int shootedArtifacts = 3;
    public boolean needVyr = false;

    boolean isFlag = false;
    @Override
    public void update() {
//        odometryClass.updateSpeed();
        cameraClass.update();

        //TODO поменял местами
//        if (odometryClass.getRobotCurVelocity().length() < 30 && Math.toRadians(odometryClass.getEncHeadVel()) < Math.toRadians(20))
//        {
//            if (cameraClass.tagState == CameraClass.TagState.Detected) {
//                odometryClass.setPos(cameraClass.getLastRecordedPosition2D());
//            }else odometryClass.updatePoses();
//        }else {
//            odometryClass.updatePoses();
//        }

//        calcRange();
//        calcAngle();
//        calcDeltaAngle();
//        getNearestFirePos();
//
//        //TODO подправить позиции на стрельбу и скорость увеличить
//        switch (GeneralInformation.current.programName){
//            case TeleOp:
//                switch (generalState){
//                    case Test:
//                        targetPos = new Position2D(0, 0, deltaAngle);
//                        driveArgs = new Args.DriveArgs(targetPos, 50);
//
//                        generalState = GeneralState.EXECUTE_IN_PROCCESS;
//                        break;
//                    case Get_pos:
//                        switch (autoState){
//                            case FIND_AND_GO_TO_FIRE_POS:
//                                targetPos = getNearestFirePos();
//                                driveArgs = new Args.DriveArgs(targetPos, 30);
//
//                                needVyr = true;
//                                generalState = GeneralState.EXECUTE_IN_PROCCESS;
//                                break;
//                            case GO_TO_LOAD_POS:
//                                switch (GeneralInformation.current.color){
//                                    case Blue:
//                                        targetPos = new Position2D(getNearestFirePos().getX(), getNearestFirePos().getY() + 30, toGlobalAngle(90));
//                                        driveArgs = new Args.DriveArgs(targetPos, 30);
//
//                                        needVyr = false;
//                                        generalState = GeneralState.EXECUTE_IN_PROCCESS;
//                                        break;
//                                    case Red:
//                                        targetPos = new Position2D(getNearestFirePos().getX(), getNearestFirePos().getY() - 30, toGlobalAngle(-90));
//                                        driveArgs = new Args.DriveArgs(targetPos, 30);
//
//                                        needVyr = false;
//                                        generalState = GeneralState.EXECUTE_IN_PROCCESS;
//                                        break;
//                                }
//                                break;
//                        }
//                        break;
//                    case EXECUTE_IN_PROCCESS:
//                        break;
//                    case STOP:
//                        break;
//                }
//                break;
//            case Auto:
//                switch (generalState){
//                    case Test:
//                        targetPos = new Position2D(0, 0, deltaAngle);
//                        driveArgs = new Args.DriveArgs(targetPos, 50);
//
//                        generalState = GeneralState.EXECUTE_IN_PROCCESS;
//                        break;
//                    case Get_pos:
//                        switch (autoState){
//                            case CHECK_READINESS_FOR_START:
//                                if(cameraClass.randomizeStatus == CameraClass.RandomizeStatus.UnDetected && cameraClass.onceSeen){
//                                    switch (GeneralInformation.current.color){
//                                        case Blue:
//                                            switch (GeneralInformation.current.startPos){
//                                                case Near_wall:
//                                                    targetPos = new Position2D(odometryClass.getEncGlobalPosition2D().getX(), odometryClass.getEncGlobalPosition2D().getY(), toGlobalAngle(-45));
//                                                    driveArgs = new Args.DriveArgs(targetPos, 20);
//
//                                                    generalState = GeneralState.EXECUTE_IN_PROCCESS;
//
//                                                    setUpToDown();
//                                                    break;
//                                                case Far_from_wall:
//                                                    targetPos = new Position2D(odometryClass.getEncGlobalPosition2D().getX(), odometryClass.getEncGlobalPosition2D().getY(), toGlobalAngle(0));
//                                                    driveArgs = new Args.DriveArgs(targetPos, 20);
//
//                                                    generalState = GeneralState.EXECUTE_IN_PROCCESS;
//
//                                                    setDownToUp();
//                                                    break;
//                                            }
//                                            break;
//                                        case Red:
//                                            switch (GeneralInformation.current.startPos){
//                                                case Near_wall:
//                                                    targetPos = new Position2D(odometryClass.getEncGlobalPosition2D().getX(), odometryClass.getEncGlobalPosition2D().getY(), toGlobalAngle(45));
//                                                    driveArgs = new Args.DriveArgs(targetPos, 20);
//
//                                                    generalState = GeneralState.EXECUTE_IN_PROCCESS;
//
//                                                    setUpToDown();
//                                                    break;
//                                                case Far_from_wall:
//                                                    targetPos = new Position2D(odometryClass.getEncGlobalPosition2D().getX(), odometryClass.getEncGlobalPosition2D().getY(), toGlobalAngle(0));
//                                                    driveArgs = new Args.DriveArgs(targetPos, 20);
//
//                                                    generalState = GeneralState.EXECUTE_IN_PROCCESS;
//
//                                                    setDownToUp();
//                                                    break;
//                                            }
//                                            break;
//                                    }
//                                } else if (cameraClass.randomizeStatus == CameraClass.RandomizeStatus.Detected && !cameraClass.onceSeen) {
//                                    switch (GeneralInformation.current.color){
//                                        case Red:
//                                            switch (GeneralInformation.current.startPos){
//                                                case Near_wall:
//                                                    targetPos = new Position2D(odometryClass.getEncGlobalPosition2D().getX(), odometryClass.getEncGlobalPosition2D().getY(), -Math.toRadians(30));
//                                                    driveArgs = new Args.DriveArgs(targetPos, 20);
//                                                    generalState = GeneralState.EXECUTE_IN_PROCCESS;
//
//                                                    setUpToDown();
//                                                    break;
//                                                case Far_from_wall:
//                                                    targetPos = new Position2D(odometryClass.getEncGlobalPosition2D().getX(), odometryClass.getEncGlobalPosition2D().getY(), -Math.toRadians(30));
//                                                    driveArgs = new Args.DriveArgs(targetPos, 20);
//                                                    generalState = GeneralState.EXECUTE_IN_PROCCESS;
//
//                                                    setDownToUp();
//                                                    break;
//                                            }
//                                            break;
//                                        case Blue:
//                                            switch (GeneralInformation.current.startPos){
//                                                case Near_wall:
//                                                    targetPos = new Position2D(odometryClass.getEncGlobalPosition2D().getX(), odometryClass.getEncGlobalPosition2D().getY(), Math.toRadians(30));
//                                                    driveArgs = new Args.DriveArgs(targetPos, 20);
//
//                                                    generalState = GeneralState.EXECUTE_IN_PROCCESS;
//
//                                                    setUpToDown();
//                                                    break;
//                                                case Far_from_wall:
//                                                    targetPos = new Position2D(odometryClass.getEncGlobalPosition2D().getX(), odometryClass.getEncGlobalPosition2D().getY(), Math.toRadians(30));
//                                                    driveArgs = new Args.DriveArgs(targetPos, 20);
//                                                    generalState = GeneralState.EXECUTE_IN_PROCCESS;
//
//                                                    setDownToUp();
//                                                    break;
//                                            }
//                                            break;
//                                    }
//                                }else {
//                                    generalState = GeneralState.DONE;
//                                    switch (GeneralInformation.current.color){
//                                        case Red:
//                                            switch (GeneralInformation.current.startPos){
//                                                case Near_wall:
//                                                    setUpToDown();
//                                                    break;
//                                                case Far_from_wall:
//                                                    setDownToUp();
//                                                    break;
//                                            }
//                                            break;
//                                        case Blue:
//                                            switch (GeneralInformation.current.startPos){
//                                                case Near_wall:
//                                                    setUpToDown();
//                                                    break;
//                                                case Far_from_wall:
//                                                    setDownToUp();
//                                                    break;
//                                            }
//                                            break;
//                                    }
//                                }
//                                break;
//                            case FIND_AND_GO_TO_FIRE_POS:
//                                targetPos = getNearestFirePos();
//                                driveArgs = new Args.DriveArgs(targetPos, 30);
//
//                                needVyr = true;
//                                generalState = GeneralState.EXECUTE_IN_PROCCESS;
//                                break;
//
//                            case GO_AFORE_ARTIFACTS:
//                                Position2D forwardArtifact = new Position2D();
//                                switch (GeneralInformation.current.color){
//                                    case Blue:
//                                        forwardArtifact = new Position2D(getArtifactPos().getX(), getArtifactPos().getY() + 20, getArtifactPos().getHeading());
//                                        break;
//                                    case Red:
//                                        forwardArtifact = new Position2D(getArtifactPos().getX(), getArtifactPos().getY() - 20, getArtifactPos().getHeading());
//                                        break;
//                                }
//                                needVyr = false;
//                                driveArgs = new Args.DriveArgs(forwardArtifact, 30);
//                                generalState = GeneralState.EXECUTE_IN_PROCCESS;
//                                break;
//
//                            case FIND_AND_GO_TO_ARTIFACTS:
//                                Position2D artifactPos = getArtifactPos();
//
//                                needVyr = false;
//
//                                driveArgs = new Args.DriveArgs(artifactPos, 30);
//
//                                generalState = GeneralState.EXECUTE_IN_PROCCESS;
//                                break;
//                        }
//                        break;
//                    case EXECUTE_IN_PROCCESS:
//                        break;
//                    case DONE:
//                        break;
//                    case DELETE_ARTIFACT:
//                        deleteArtifact();
//                        break;
//                    case STOP:
//                        break;
//                    case EMERGENCY_RATTLING:
//                        break;
//
//                }
//
//                break;
//        }
    }

    private void setUpToDown(){
        minI = 2;
        movementLogic = MovementLogic.Up_to_down;
    }
    private void setDownToUp(){
        minI = 8;
        movementLogic = MovementLogic.Down_to_up;
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
        shootedArtifacts++;
    }
    public enum MovementLogic{
        Up_to_down,
        Down_to_up
    }

    public Position2D getArtifactPos() {
        Position2D artifactsPos = new Position2D();

        switch (movementLogic){
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
            case Down_to_up:
                if(minI == 0 && GeneralInformation.current.generalObjects.getClosestArtifacts()[minI][0] == 0){
                    break;
                }

                if(GeneralInformation.current.generalObjects.getClosestArtifacts()[minI][0] == 0){
                    minI --;
                    break;
                }
                break;
        }

        artifactsPos.setX(GeneralInformation.current.generalObjects.getClosestArtifacts()[minI][1]);
        artifactsPos.setY(GeneralInformation.current.generalObjects.getClosestArtifacts()[minI][2]);
        artifactsPos.setHeading(GeneralInformation.current.generalObjects.getClosestArtifacts()[minI][4]);

        //TODO Если что подправить угол
        return new Position2D(artifactsPos.getX(), artifactsPos.getY(), artifactsPos.getHeading());
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
        telemetry.addData("ShootedArtifacts", shootedArtifacts);

//        telemetry.addData("Founded Robot Angle", "%.1f°", foundedAngle * RAD);
        telemetry.addData("Target angle", "%.1f°", deltaAngle * RAD);
        telemetry.addData("Range to target", "%.1f cm", range);

        if(autoState == AutoState.FIND_AND_GO_TO_ARTIFACTS) {
            telemetry.addData("minI", minI);
            telemetry.addData("Artifact", "Color: %s X %s Y %s H %s",GeneralInformation.current.generalObjects.getClosestArtifacts()[minI][0],GeneralInformation.current.generalObjects.getClosestArtifacts()[minI][1], GeneralInformation.current.generalObjects.getClosestArtifacts()[minI][2], GeneralInformation.current.generalObjects.getClosestArtifacts()[minI][4]);
            telemetry.addData("Artifacts pos", "X %s Y %s H %s",targetPos.getX(), targetPos.getY(), targetPos.getHeading());
        }
        telemetry.addLine();

        cameraClass.showData();
        odometryClass.showData();
    }
}
