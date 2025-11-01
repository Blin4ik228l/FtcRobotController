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
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.Modules.Module;
import org.firstinspires.ftc.teamcode.Robot.Odometry.Parts.MathUtils.Vector2;
import org.firstinspires.ftc.teamcode.TeamColor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;


import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.InputStream;
import java.util.Map;
import java.util.Objects;
import java.util.concurrent.TimeUnit;

public class CameraClass extends Module{
    public CameraClass(OpMode op, TeamColor teamColor)  {
        super(op.telemetry);

        this.teamColor = teamColor;

        webcamName = op.hardwareMap.get(WebcamName.class, "Webcam 1");
//        -9,-13,-20
        cameraPosition = new Position(DistanceUnit.CM,0 ,10,5, 0);//Позиция камеры относительно координат робота
        cameraOrientation = new YawPitchRollAngles(AngleUnit.RADIANS, 0, Math.toRadians(-75), Math.toRadians(1), 0);//На сколько камера повёрнута относительно неё же
//        694.068,694.068,313.099, 236.335
//        1426.5,1426.5,627.916, 353.73
//        1505.6234281835175, 1453.6892287156643, 656.9498728834548, 326.8476937970202
//        820.147f, 820.147f, 311.822f, 268.136f
//        708.013f, 708.013f, 311.973, 253.313f 190
//        814.732f, 814.732f, 385.096f, 312.409f 120
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

        while (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING){}

        exposure = visionPortal.getCameraControl(ExposureControl.class);
        exposure.setMode(ExposureControl.Mode.Manual);//Если камера не поддерживает настройку экспозиции
        exposure.setExposure(5, TimeUnit.MILLISECONDS);//Экспозиция

        gain = visionPortal.getCameraControl(GainControl.class);
        gain.setGain(190);//яркость

        aprilTagProcessor.setDecimation(2);
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
    private boolean isPosWasTaken = false;
    private boolean isObeliskWasSeen = false;
    private boolean isStopStreaming = false;

    private boolean isRobotStoped = true;
    private boolean offTakingPos = false;
    public double rad = 180 / Math.PI;
    public double cameraFov = Math.toRadians(60);

    public int countTag = 0;

    public org.firstinspires.ftc.teamcode.Robot.Odometry.Parts.MathUtils.Position lastRecordedPos = null;
    private org.firstinspires.ftc.teamcode.Robot.Odometry.Parts.MathUtils.Position robotPosition;
    public void execute() {
        boolean isEmpty = aprilTagProcessor.getDetections().isEmpty();

        int numTags = aprilTagProcessor.getDetections().size();

        if(!isRobotStoped){
            countTag = 0;
        }
        if(countTag == numTags){
            countTag = 0;
        }

        if(isStopStreaming){
            return;
        }
        if(isObeliskWasSeen && offTakingPos && isRobotStoped){
            stopStreaming();
            isStopStreaming = true;
        }
        if(!isEmpty) {
            detection = aprilTagProcessor.getDetections().get(countTag);

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
            if (lastRecordedPos != null){
                offTakingPos  = true;
            }
            if(id == 20 || id == 24){
                isTagWasSeen = true;
                tagOutOfRange = false;
                isPosWasTaken = true;

                robotFieldX = detection.robotPose.getPosition().x;
                robotFieldY = detection.robotPose.getPosition().y;
                robotFieldZ = detection.robotPose.getPosition().z;

//                robotFieldX = detection.ftcPose.x;
//                robotFieldY = detection.ftcPose.y;
//                robotFieldZ = detection.ftcPose.z;

                robotFieldPitch = detection.robotPose.getOrientation().getPitch(AngleUnit.RADIANS);
                robotFieldRoll  = detection.robotPose.getOrientation().getRoll(AngleUnit.RADIANS);
                robotFieldYaw   = detection.robotPose.getOrientation().getYaw(AngleUnit.RADIANS);

//                robotFieldPitch = detection.ftcPose.pitch;
//                robotFieldRoll  = detection.ftcPose.roll;
//                robotFieldYaw   = detection.ftcPose.yaw;

                tagXFromRobot = teamColor.getWallCoord(id)[0] - robotFieldX;
                tagYFromRobot = teamColor.getWallCoord(id)[1] - robotFieldY;
                tagZFromRobot = teamColor.getWallCoord(id)[2] - robotFieldZ;

                lastRecordedPos = new org.firstinspires.ftc.teamcode.Robot.Odometry.Parts.MathUtils.Position(robotFieldX, robotFieldY, robotFieldYaw);

                robotRangeToTag = Math.hypot(tagZFromRobot, Math.hypot(tagXFromRobot, tagYFromRobot));
                cameraElevation = Math.acos(tagZFromRobot / robotRangeToTag);
                cameraBearing   = Math.acos(tagYFromRobot / robotRangeToTag);
            }
            countTag += 1;
        }


    }
    public boolean isStopStreaming(){
        return isStopStreaming;
    }
    public void activityCentre(){
//        if(tagOutOfRange && isTagWasSeen && isObeliskWasSeen ){
//            stopStreaming();
//            isTagShouldBeInFov();
//        }
//        if(!tagOutOfRange && isTagWasSeen && isObeliskWasSeen ){
//            resumeStreaming();
//        }
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
    public void setRobotVeloFromOdometry(Vector2 robotVel){
        isRobotStoped = robotVel.length() == 0;
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

    public boolean isPosWasTaken(){
        return isPosWasTaken;
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
        telemetry.addLine(String.format("\nXYZ %6.2f %6.2f %6.2f", robotFieldX, robotFieldY, robotFieldZ));

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
