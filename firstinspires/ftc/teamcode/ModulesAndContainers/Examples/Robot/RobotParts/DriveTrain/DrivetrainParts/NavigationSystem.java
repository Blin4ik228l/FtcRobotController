package org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.DriveTrain.DrivetrainParts;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.TaskAndArgs.Args;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.GeneralInformation;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.DriveTrain.DrivetrainParts.Odometry.Odometry;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.DriveTrain.DrivetrainParts.Odometry.OdometryData;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.DriveTrain.DrivetrainParts.Odometry.Parts.MathUtils.Position2D;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.DriveTrain.DrivetrainParts.Odometry.Parts.MathUtils.Vector2;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Modules.Extenders.UpdatableModule;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Modules.Module;

import java.util.ArrayList;
import java.util.Arrays;

public class NavigationSystem extends UpdatableModule{
    private final Odometry odometry;
    private final PositionController positionController;
    private GeneralInformation generalInformation;

    public NavigationSystem(OpMode op) {
        super(op);

        this.odometry = new Odometry(op);
        this.positionController = new PositionController(op);
    }

    public void setGeneralInformation(GeneralInformation generalInformation) {
        this.generalInformation = generalInformation;
    }

    @Override
    public void update() {
        positionController.update();
    }

    @Override
    public void showData() {
        odometry.showData();
    }
    public OdometryData currentData(){
        return odometry.odometryBuffer.read();
    }
    public OdometryData targetData(){
        return positionController.path;
    }
    public class PositionController extends UpdatableModule {

        public PositionController(OpMode op) {
            super(op);
        }
        public int minI;
        public int mx = 2, mn = -1;
        public int shootedArtifacts = 3;

        private OdometryData path;
        public ArrayList<OdometryData[]> pathBuilder;
        @Override
        public void update() {

        }
        public void getData(){
            pathBuilder.get(0);
        }

        public void switchPose(){
            pathBuilder.remove(0);
        }

        public double toGlobalAngle(double angle){
            return Math.toRadians(angle) - Math.toRadians(90);
        }
        public void deleteArtifact(){
            if (generalInformation.startPos == GeneralInformation.StartPos.Far_from_wall){
                generalInformation.generalObjects.getClosestArtifacts().get(9)[0] = 0;
                generalInformation.generalObjects.getClosestArtifacts().remove(9);
            }else {
                generalInformation.generalObjects.getClosestArtifacts().get(0)[0] = 0;
                generalInformation.generalObjects.getClosestArtifacts().remove(0);
            }
            shootedArtifacts++;
        }

        public Position2D getArtifactPos() {
            double[] artifactData;

            //TODO
            if (generalInformation.startPos == GeneralInformation.StartPos.Far_from_wall){
                artifactData = generalInformation.generalObjects.getClosestArtifacts().get(9);
            }else {
                artifactData = generalInformation.generalObjects.getClosestArtifacts().get(0);
            }

            return new Position2D(artifactData[1], artifactData[2], artifactData[4]);
        }

        public Position2D getNearestFirePos(){
            Position2D firePos;
            Vector2 nearDelta = new Vector2(generalInformation.generalObjects.getFireZones()[0][0] - currentData().getRobotPosition().getX(), generalInformation.generalObjects.getFireZones()[0][1] - currentData().getRobotPosition().getY());
            Vector2 farDelta = new Vector2(generalInformation.generalObjects.getFireZones()[1][0] - currentData().getRobotPosition().getX(), generalInformation.generalObjects.getFireZones()[1][1] - currentData().getRobotPosition().getY());

            //TODO Если наложиться угол
            if(nearDelta.length() > farDelta.length()){
                firePos = new Position2D(generalInformation.generalObjects.getFireZones()[1][0], generalInformation.generalObjects.getFireZones()[1][1], currentData().getRobotPosition().getHeading());
            }else firePos = new Position2D(generalInformation.generalObjects.getFireZones()[0][0], generalInformation.generalObjects.getFireZones()[0][1], currentData().getRobotPosition().getHeading());

            return firePos;
        }
        public Position2D getBasePos(){
            return new Position2D(generalInformation.generalObjects.getBaseCoord()[0], generalInformation.generalObjects.getBaseCoord()[1], Math.toRadians(90));
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
                telemetry.addData("Artifact", "Color: %s X %s Y %s H %s",generalInformation.generalObjects.getClosestArtifacts()[minI][0],generalInformation.generalObjects.getClosestArtifacts()[minI][1], GeneralInformation.current.generalObjects.getClosestArtifacts()[minI][2], GeneralInformation.current.generalObjects.getClosestArtifacts()[minI][4]);
                telemetry.addData("Artifacts pos", "X %s Y %s H %s",targetPos.getX(), targetPos.getY(), targetPos.getHeading());
            }
            telemetry.addLine();
        }
    }

}
