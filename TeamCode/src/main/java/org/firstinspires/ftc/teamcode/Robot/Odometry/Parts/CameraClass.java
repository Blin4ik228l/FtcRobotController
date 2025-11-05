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
import org.firstinspires.ftc.teamcode.Robot.Odometry.ExOdometry;
import org.firstinspires.ftc.teamcode.TeamColor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;


import java.util.concurrent.TimeUnit;

public class CameraClass extends Module{
    public CameraClass(OpMode op, TeamColor teamColor, ExOdometry exOdometry)  {
        super(op.telemetry);

        this.exOdometry = exOdometry;
        this.teamColor = teamColor;

        webcamName = op.hardwareMap.get(WebcamName.class, "Webcam 1");

        cameraPosition = new Position(DistanceUnit.CM,0 ,20.1628,26.086, 0);//Позиция камеры относительно координат робота
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

        while (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING){}

        exposure = visionPortal.getCameraControl(ExposureControl.class);
        exposure.setMode(ExposureControl.Mode.Manual);//Если камера не поддерживает настройку экспозиции
        exposure.setExposure(5, TimeUnit.MILLISECONDS);//Экспозиция

        gain = visionPortal.getCameraControl(GainControl.class);
        gain.setGain(190);//яркость

        aprilTagProcessor.setDecimation(2);

        telemetry.addLine("Camera Inited");
    }
    private final Position cameraPosition;
    private final YawPitchRollAngles cameraOrientation;
    private final ExOdometry exOdometry;
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
    private boolean isAllianceTagWasSeen = false;
    private boolean isObeliskWasSeen = false;
    private boolean isRobotHaveMinVel = true;
    private boolean isPosWasWritten = false;
    public double rad = 180 / Math.PI;
    public double cameraFov = Math.toRadians(60);
    public int countTag = 0;
    public void execute() {
        if(!isPosWasWritten){
            updatePos();
        }

        if(isPosShouldBeTaken() && isPosWasWritten){
//            resumeStreaming();
            updatePos();
            writePos();
        }else{
//            stopStreaming();
        }

        if(isAllianceTagWasSeen && !isPosWasWritten && isObeliskWasSeen) {
            isPosWasWritten = true;
            writePos();
//            stopStreaming();
        }

    }
    public boolean isPosShouldBeTaken(){
        return isRobotHaveMinVel() &&  isTagShouldBeInFov() && isRobotInNeededRange() ;
    }
    public void updatePos(){
        boolean isEmpty = aprilTagProcessor.getDetections().isEmpty();

        int numTags = aprilTagProcessor.getDetections().size();

        if(!isRobotHaveMinVel){
            countTag = 0;
        }
        if(countTag == numTags){
            countTag = 0;
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

            if(id == 20 || id == 24){
                isAllianceTagWasSeen = true;

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
            countTag += 1;
        }
    }
    public void writePos(){
        exOdometry.encGlobalPosition.setX(robotFieldX);
        exOdometry.encGlobalPosition.setY(robotFieldY);
        exOdometry.encGlobalPosition.setHeading(robotFieldYaw);
    }
    public boolean isTagShouldBeInFov(){
        double leftBorder = exOdometry.encGlobalPosition.getHeading() + (Math.signum(exOdometry.encGlobalPosition.getHeading()) * cameraFov / 4.0);
        double rightBorder = exOdometry.encGlobalPosition.getHeading() - (Math.signum(exOdometry.encGlobalPosition.getHeading()) * cameraFov / 4.0);

        return !((Math.toRadians(44) < leftBorder && rightBorder < Math.toRadians(44) && exOdometry.encGlobalPosition.getY() > 0)
                        ||
                (Math.toRadians(136) < (leftBorder) && (rightBorder) < Math.toRadians(136)) && exOdometry.encGlobalPosition.getY() < 0);
    }
    public boolean isRobotInNeededRange(){
        double targX = -(2.54 * 58.3727) - exOdometry.encGlobalPosition.getX();
        double targY = Math.signum(exOdometry.encGlobalPosition.getY()) * (2.54 * 55.6425) - exOdometry.encGlobalPosition.getY();

        double range = Math.hypot(targY, targX);

        return range <= 200 && range >= 40;
    }
    public boolean isRobotHaveMinVel(){
       return isRobotHaveMinVel = exOdometry.robotSelfCentricVel.length() <= 5;
    }

    public void stopStreaming(){
        visionPortal.stopStreaming();
    }
    public void resumeStreaming(){
        visionPortal.resumeStreaming();
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
