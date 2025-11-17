package org.firstinspires.ftc.teamcode.Robot.RobotParts;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Modules.UpdatableModule;
import org.firstinspires.ftc.teamcode.Robot.RobotParts.DrivetrainParts.Odometry.Parts.MathUtils.Position2D;
import org.firstinspires.ftc.teamcode.Robot.RobotParts.DrivetrainParts.Odometry.Parts.MathUtils.Vector2;
import org.firstinspires.ftc.teamcode.TeamColor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;


import java.util.concurrent.TimeUnit;

public class CameraClass extends UpdatableModule {
    public CameraClass(OpMode op, TeamColor teamColor)  {
        super(op.telemetry);

        this.teamColor = teamColor;

        webcamName = op.hardwareMap.get(WebcamName.class, "Webcam 1");

        cameraPosition = new Position(DistanceUnit.CM,-25 ,20.1628,26.086, 0);//Позиция камеры относительно координат робота
        cameraOrientation = new YawPitchRollAngles(AngleUnit.RADIANS, Math.toRadians(180), 0, 0, 0);//На сколько камера повёрнута относительно неё же

        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(false)
                .setDrawCubeProjection(false)
                .setDrawTagID(true)
                .setDrawTagOutline(false)
                .setLensIntrinsics(708.013f, 708.013f, 311.973, 253.313f)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getDecodeTagLibrary())
                .setOutputUnits(DistanceUnit.CM, AngleUnit.RADIANS)
                .setCameraPose(cameraPosition, cameraOrientation)
                .build();

        visionPortal = new VisionPortal.Builder()
                .addProcessor(aprilTagProcessor)
                .setCamera(webcamName)
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .build();

        while (true){
            if (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
                isCameraStreaming = true;
                break;}
        }

        exposure = visionPortal.getCameraControl(ExposureControl.class);
        exposure.setMode(ExposureControl.Mode.Manual);//Если камера не поддерживает настройку экспозиции
        exposure.setExposure(5, TimeUnit.MILLISECONDS);//Экспозиция

        gain = visionPortal.getCameraControl(GainControl.class);
        gain.setGain(190);//яркость

        aprilTagProcessor.setDecimation(2);

        lastPosWasTaked = new ElapsedTime();
        cameraState = CameraState.noDetected;

        telemetry.addLine("Camera Inited");
    }
    private final Position cameraPosition;
    private final YawPitchRollAngles cameraOrientation;
    private final AprilTagProcessor aprilTagProcessor;
    private final VisionPortal visionPortal;
    public AprilTagDetection detection;
    private final WebcamName webcamName;
    private final ExposureControl exposure;
    private final GainControl gain;
    public final TeamColor teamColor;
    private final int green = 1;
    private final int purple = 2;
    private int id;
    public int[] randomizedArtifact = new int[3];
    /*0 - в массиве это зелёный шар
    * 1 - в массиве это фиолетовый*/
    public double robotFieldX;
    public double robotFieldY;
    public double robotFieldZ;
    public double tagXFromRobot;
    public double tagYFromRobot;
    public double tagZFromRobot;
    public double robotFieldPitch;
    public double robotFieldRoll;
    public double robotFieldYaw;
    public double robotRangeToTag;
    public double cameraElevation;
    public double cameraBearing;
    private boolean isObeliskWasSeen = false;
    private boolean isPosWasTakenFirstly = false;
    private boolean isCameraStreaming;
    public double rad = 180 / Math.PI;
    public double cameraFov = Math.toRadians(60);
    public Position2D robotPos;
    public Vector2 robotVel;
    public double robotHeadVel;
    public double rangeFromOdometry;

    public ElapsedTime lastPosWasTaked;

    public CameraState cameraState;
    boolean state;
    public enum CameraState{
        noDetected,
        hasDetected
    }

    public void setPositionFromOdometry(Position2D robotPos){
        this.robotPos = robotPos;
    }
    public void setRobotVelFromOdometry(Vector2 robotVel, double robotHeadVel){
        this.robotVel = robotVel;
        this.robotHeadVel = robotHeadVel;
    }
    public void setRangeFromOdometry(double rangeFromOdometry){
        this.rangeFromOdometry = rangeFromOdometry;
    }
    public void setOdometryState(boolean state){
        this.state = state;
    }

    @Override
    public void update(){

        if(isObeliskWasSeen && isPosWasTakenFirstly){
            if(!isRobotHaveMinVel() || !isTagShouldBeInFov()|| !isRobotHaveMinRange() || lastPosWasTaked.seconds() < 2){
                cameraState = CameraState.noDetected;
                return;
            }
        }

        boolean isEmpty = aprilTagProcessor.getDetections().isEmpty();

        if(!isEmpty) {
            for(AprilTagDetection detection1 : aprilTagProcessor.getDetections()){
                id = detection1.id;

                if(!isObeliskWasSeen){
                    if(id == 21 || id == 22 || id == 23){
                        getRandomizedArtifactFromId(id);
                    }
                }

                if((id == 20 || id == 24) ){
                    isPosWasTakenFirstly = true;
                    calculate();
                    cameraState = CameraState.hasDetected;
                    lastPosWasTaked.reset();
                }

            }
        }
    }
    public Position2D returnWritedPos(){
        if(cameraState == CameraState.noDetected) return new Position2D();

        return new Position2D(robotFieldX, robotFieldY, robotFieldYaw);
    }


    public void getRandomizedArtifactFromId(int id) {
        if(id == 21){
            randomizedArtifact = new int[] {green, purple, purple};
            isObeliskWasSeen = true;
        }
        if(id == 22){
            randomizedArtifact = new int[] {purple, green, purple};
            isObeliskWasSeen = true;
        }
        if(id == 23){
            randomizedArtifact = new int[] {purple, purple, green};
            isObeliskWasSeen = true;
        }
    }
    public void calculate(){
        robotFieldX = detection.robotPose.getPosition().x;
        robotFieldY = detection.robotPose.getPosition().y;
        robotFieldZ = detection.robotPose.getPosition().z;

        robotFieldPitch = detection.robotPose.getOrientation().getPitch(AngleUnit.RADIANS);
        robotFieldRoll  = detection.robotPose.getOrientation().getRoll(AngleUnit.RADIANS);
        robotFieldYaw   = detection.robotPose.getOrientation().getYaw(AngleUnit.RADIANS);

        tagXFromRobot = teamColor.getWallCoord(id)[0] - robotFieldX;
        tagYFromRobot = teamColor.getWallCoord(id)[1] - robotFieldY;
        tagZFromRobot = teamColor.getWallCoord(id)[2] - robotFieldZ;

        robotRangeToTag = Math.hypot(tagZFromRobot, Math.hypot(tagXFromRobot, tagYFromRobot));
        cameraElevation = Math.acos(tagZFromRobot / robotRangeToTag);
        cameraBearing   = Math.acos(tagYFromRobot / robotRangeToTag);
    }

    public boolean isTagShouldBeInFov(){
        if(!state) return true;

        double leftBorder = robotPos.getHeading() + (Math.signum(robotPos.getHeading()) * cameraFov / 4.0);
        double rightBorder = robotPos.getHeading() - (Math.signum(robotPos.getHeading()) * cameraFov / 4.0);

        return !((Math.toRadians(44) < leftBorder && rightBorder < Math.toRadians(44) && robotPos.getY() > 0)
                        ||
                (Math.toRadians(136) < (leftBorder) && (rightBorder) < Math.toRadians(136)) && robotPos.getY() < 0);
    }
    public boolean isRobotInNeededRange(){
        double targX = -(2.54 * 58.3727) - robotPos.getX();
        double targY = Math.signum(robotPos.getY()) * (2.54 * 55.6425) - robotPos.getY();

        double range = Math.hypot(targY, targX);

        return range <= 200 && range >= 40;
    }
    public boolean isRobotHaveMinVel(){
       return robotVel.length() <= 30;
    }
    public boolean isRobotHaveMinRange(){
        if(!state) return true;

        return rangeFromOdometry < 200 && rangeFromOdometry > 40;
    }

    public String getColorFromNumber(int number){
        return number == 2 ? "Purple" : number == 1 ? "Green" : "Empty";
    }

    public void showData(){
        telemetry.addLine("=== CAMERA ===");
        telemetry.addData("Randomized artifacts: ", getColorFromNumber(randomizedArtifact[0]), getColorFromNumber(randomizedArtifact[1]), getColorFromNumber(randomizedArtifact[2]));
        telemetry.addData("Tags Found", 0);
        telemetry.addData("Streaming", isCameraStreaming);
        telemetry.addData("Robot Pos", "X:%.2f Y:%.2f Z:%.2f", robotFieldX, robotFieldY, robotFieldZ);
        telemetry.addData("Robot Angles", "R:%.1f P:%.1f Y:%.1f", robotFieldRoll * rad, robotFieldPitch * rad, robotFieldYaw * rad);
        telemetry.addLine();
    }
}
