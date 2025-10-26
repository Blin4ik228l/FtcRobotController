package org.firstinspires.ftc.teamcode.Robot.Odometry.Parts;

import android.annotation.SuppressLint;
import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Modules.Module;
import org.firstinspires.ftc.teamcode.TeamColor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.concurrent.TimeUnit;

public class CameraClass extends Module{
    public CameraClass(OpMode op, TeamColor teamColor) {
        super(op.telemetry);

        this.teamColor = teamColor;

        webcamName = op.hardwareMap.get(WebcamName.class, "Webcam 1");

        cameraPosition = new Position(DistanceUnit.CM, 0, 11, 5, 0);//Позиция камеры относительно координат робота
        cameraOrientation = new YawPitchRollAngles(AngleUnit.RADIANS, 0, Math.toRadians(-45), 0, 0);//На сколько камера повёрнута относительно неё же

        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(false)
                .setDrawCubeProjection(false)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
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

        while (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING){}

        exposure = visionPortal.getCameraControl(ExposureControl.class);
        exposure.setMode(ExposureControl.Mode.Manual);//Если камера не поддерживает настройку экспозиции
        exposure.setExposure(5, TimeUnit.MILLISECONDS);//Экспозиция

        gain = visionPortal.getCameraControl(GainControl.class);
        gain.setGain(190);//яркость

        aprilTagProcessor.setDecimation(3);
    }
    private final Position cameraPosition;
    private final YawPitchRollAngles cameraOrientation;
    private final AprilTagProcessor aprilTagProcessor;
    private final VisionPortal visionPortal;
    private final WebcamName webcamName;
    private final ExposureControl exposure;
    private final GainControl gain;
    public final TeamColor teamColor;
    private final int green = 0;
    private final int purple = 1;
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
    public AprilTagDetection detection;
    private boolean tagOutOfRange = true;
    private boolean isTagOutOfFov = true;
    private boolean isTagWasSeen = false;
    private boolean isObeliskWasSeen = false;
    public double rad = 180 / Math.PI;
    public double cameraFov = Math.toRadians(60);

    private org.firstinspires.ftc.teamcode.Robot.Odometry.Parts.MathUtils.Position robotPosition;
    public void execute() {
        boolean isEmpty = aprilTagProcessor.getDetections().isEmpty();

        if(!isEmpty) {
            detection = aprilTagProcessor.getDetections().get(0);

            id = detection.id;

            if(!isObeliskWasSeen){
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
            if(id == 20 || id == 24 ){
                isTagWasSeen = true;
                tagOutOfRange = false;

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
            }else {
                tagOutOfRange = true;
                activityCentre();
            }
        }
    }
    public void activityCentre(){
        if(tagOutOfRange && isTagWasSeen && isObeliskWasSeen ){
            stopStreaming();
            isTagShouldBeInFov();
        }
        if(!tagOutOfRange && isTagWasSeen && isObeliskWasSeen ){
            resumeStreaming();
        }
    }

    public void isTagShouldBeInFov(){
        tagOutOfRange = !(robotPosition.getHeading() + Math.signum(robotPosition.getHeading()) * (cameraFov / 2.0) >= teamColor.getWallCoord(24)[3] && robotPosition.getHeading() - Math.signum(robotPosition.getHeading()) * (cameraFov / 2.0) <= teamColor.getWallCoord(24)[3]);
        if(tagOutOfRange){
            tagOutOfRange = !(robotPosition.getHeading() + Math.signum(robotPosition.getHeading()) * (cameraFov / 2.0) <= teamColor.getWallCoord(20)[3] && robotPosition.getHeading() - Math.signum(robotPosition.getHeading()) * (cameraFov / 2.0) >= teamColor.getWallCoord(20)[3]);
        }
    }

    public void setRobotPosFromOdometry(org.firstinspires.ftc.teamcode.Robot.Odometry.Parts.MathUtils.Position robotPos){
        robotPosition = robotPos;
    }

    public void stopStreaming(){
        visionPortal.stopStreaming();
    }
    public void resumeStreaming(){
        visionPortal.resumeStreaming();
    }

    public boolean isTagOutOfRange(){
        return tagOutOfRange;
    }

    public void showFoundTagId(){
        telemetry.addData("Tag ID", id);
    }

    public void showExposureSupport(){
        telemetry.addData("isExposure supported", exposure.isExposureSupported());
    }
    public void showRandomizedArtifacts(){
        telemetry.addLine("Randomized Artifacts")
                .addData("",  randomizedArtifact[0] == green ? "Green" : "Purple", "|")
                .addData("",  randomizedArtifact[1] == green ? "Green" : "Purple", "|")
                .addData("",  randomizedArtifact[2] == green ? "Green" : "Purple");
        telemetry.addLine();
    }

    @SuppressLint("DefaultLocale")
    public void showRobotPosition(){
        telemetry.addLine(String.format("\nXYZ %6.2f %6.2f ", robotFieldX, robotFieldY, 0));

        telemetry.addLine("Robot angle on field")
                .addData("\nroll", robotFieldRoll * rad)//Угол вокруг Н
                .addData("\npitch", robotFieldPitch * rad)//Угол вокруг X
                .addData("yaw", robotFieldYaw * rad); //Угол вокруг Z
        telemetry.addLine();
    }

    public void showCameraREB(){
        telemetry.addLine("Camera REB")
                .addData("\nR", robotRangeToTag)
                .addData("\nE", cameraElevation * rad)//Угол наклонёности камеры
                .addData("B", cameraBearing * rad); //Угол отклонёности камеры
        telemetry.addLine("R - Range, E - Elevation, B - Bearing");
    }
}
