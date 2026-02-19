package org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Players.Pl2;

import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.DriveTrain.DrivetrainParts.NavigationSystem;

public class FireMath {
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

    public VyrState vyrState;

    public NavigationSystem.PositionController.MovementLogic movementLogic;
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

}
