package org.firstinspires.ftc.teamcode.Robot.Odometry.Parts;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Modules.Module;
import org.firstinspires.ftc.teamcode.TeamColor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.concurrent.TimeUnit;

public class CameraClass extends Module{
    public CameraClass(OpMode op, TeamColor teamColor) {
        super(op.telemetry);

        this.teamColor = teamColor;

        webcamName = op.hardwareMap.get(WebcamName.class, "Webcam 1");

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
    Position cameraPosition = new Position(DistanceUnit.CM, 0, 0, -15, 0);
    YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.RADIANS, 0, Math.toRadians(-45), 0, 0);
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

    public double myX ;
    public double myY ;
    double myZ ;

    double myPitch ;
    double myRoll ;
   public double myYaw ;

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
    public double[] blueWallCoord = new double[]{
            2.54 * (-58.3727),//X
            2.54 * (-55.6425),//Y
            2.54 * (29.5)//Z
    };

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

                 myX = detection.robotPose.getPosition().x;
                 myY = detection.robotPose.getPosition().y;
                 myZ = -detection.robotPose.getPosition().z - 21 ;
//
                 myPitch = detection.robotPose.getOrientation().getPitch(AngleUnit.DEGREES);
                 myRoll = detection.robotPose.getOrientation().getRoll(AngleUnit.DEGREES);
                 myYaw = detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES);
            }

            telemetry.addLine(String.format("\nXYZ %6.2f %6.2f ", myX, myY, 0));
            telemetry.addData("\nroll", myRoll);
            telemetry.addData("\npitch",myPitch );
            telemetry.addData("\nyaw", myYaw);
        }
//        telemetry.addData("Randomized Artifacts",  randomizedArtifact[0] == green ? "Green" : "Purple", "|");
//        telemetry.addData("Randomized Artifacts",  randomizedArtifact[1] == green ? "Green" : "Purple", "|");
//        telemetry.addData("Randomized Artifacts",  randomizedArtifact[2] == green ? "Green" : "Purple");

        telemetry.addData("isExposure supported", exposure.isExposureSupported());

//        telemetry.addData("\nrollftc", ftcRoll* 180/Math.PI);
//        telemetry.addData("\npitchftc", ftcPitch* 180/Math.PI);
//        telemetry.addData("\nyawftc", ftcYaw* 180/Math.PI);
//
//        telemetry.addData("\nbearing", bearing* 180/Math.PI);
//        telemetry.addData("\nelevation", elevation* 180/Math.PI);
//        telemetry.addData("\nrange", range);
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
