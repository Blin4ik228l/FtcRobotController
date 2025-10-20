package org.firstinspires.ftc.teamcode.Robot.Odometry.Parts;

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

        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setOutputUnits(DistanceUnit.CM, AngleUnit.RADIANS)
                .setTagLibrary(AprilTagGameDatabase.getDecodeTagLibrary())
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
    public AprilTagProcessor aprilTagProcessor;
    public VisionPortal visionPortal;
    public WebcamName webcamName;
    public ExposureControl exposure;
    public GainControl gain;
    public TeamColor teamColor;

    public int[] randomizedArtifact = new int[3];
    /*0 - в массиве это зелёный шар
    * 1 - в массиве это фиолетовый*/

    public int green = 0;
    public int purple = 1;

    double myX ;
    double myY ;
    double myZ ;

    double myPitch ;
    double myRoll ;
    double myYaw ;

   public double ftcX;
   public double ftcY ;
   public double ftcZ ;

    double ftcPitch ;
    public double ftcRoll ;
   public double ftcYaw ;

   public double bearing;
   public double elevation;
   public double range;

    double lastFtcYaw = 0;
    double deltaFtcYaw = 0;
    int id;
    AprilTagDetection detection;
    public void execute() {


        boolean isEmpty = aprilTagProcessor.getDetections().isEmpty();

        if (!isEmpty) {
            detection = aprilTagProcessor.getDetections().get(0);
            id = detection.id;

            if(id == 21){
                randomizedArtifact = new int[] {green, purple, purple};
            }
            if(id == 22){
                randomizedArtifact = new int[] {purple, green, purple};
            }
            if(id == 23){
                randomizedArtifact = new int[] {purple, purple, green};
            }
            telemetry.addData("Tag ID", id);

            if(id == teamColor.getTagIds()[0]){
                Position cameraPosition = new Position(DistanceUnit.CM, 0, 0, 0, 0);
                YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.RADIANS, 0, Math.toRadians(-90), 0, 0);



//                 myX = detection.robotPose.getPosition().x ;
//                 myY = detection.robotPose.getPosition().y ;
//                 myZ = detection.robotPose.getPosition().z ;
//
//                 myPitch = detection.robotPose.getOrientation().getPitch(AngleUnit.RADIANS);
//                 myRoll = detection.robotPose.getOrientation().getRoll(AngleUnit.RADIANS);
//                 myYaw = detection.robotPose.getOrientation().getYaw(AngleUnit.RADIANS);

                 ftcX = detection.ftcPose.x;
                 ftcY = detection.ftcPose.y;
                 ftcZ = detection.ftcPose.z;

                 ftcPitch = detection.ftcPose.pitch;
                 ftcRoll = detection.ftcPose.roll;
                 ftcYaw = detection.ftcPose.yaw;

                 bearing = detection.ftcPose.bearing;
                elevation = detection.ftcPose.elevation;
                range = detection.ftcPose.range;
            }
//            randomizedArtifact[0] == green ? "Green" : "Purple"

        }
        deltaFtcYaw = ftcYaw - lastFtcYaw;
        lastFtcYaw = ftcYaw;

        telemetry.addData("Randomized Artifacts",  randomizedArtifact[0] == green ? "Green" : "Purple", "|");
        telemetry.addData("Randomized Artifacts",  randomizedArtifact[1] == green ? "Green" : "Purple", "|");
        telemetry.addData("Randomized Artifacts",  randomizedArtifact[2] == green ? "Green" : "Purple");

//        telemetry.addLine(String.format("\nXYZ %6.2f %6.2f %6.2f", myX, myY, myZ));
        telemetry.addLine(String.format("\nXYZftc %6.2f %6.2f %6.2f",ftcX , ftcY, ftcZ));

        telemetry.addData("isExposure supported", exposure.isExposureSupported());

//        telemetry.addData("\nroll", myPitch * 180/Math.PI);
//        telemetry.addData("\npitch", myRoll* 180/Math.PI);
//        telemetry.addData("\nyaw", myYaw* 180/Math.PI);

        telemetry.addData("\nrollftc", ftcRoll* 180/Math.PI);
        telemetry.addData("\npitchftc", ftcPitch* 180/Math.PI);
        telemetry.addData("\nyawftc", ftcYaw* 180/Math.PI);

        telemetry.addData("\nbearing", bearing* 180/Math.PI);
        telemetry.addData("\nelevation", elevation* 180/Math.PI);
        telemetry.addData("\nrange", range);
        telemetry.addLine();
    }

    public double getYaw(){
        return ftcYaw;
    }
    public double getRoll(){
        return ftcRoll;
    }
    public double getPitch(){
        return ftcPitch;
    }
    public double getDeltaFtcYaw(){
        return deltaFtcYaw;
    }
}
