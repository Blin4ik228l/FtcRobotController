package org.firstinspires.ftc.teamcode.MainParts.Examples.Robot.RobotParts;

import android.util.Size;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.MainParts.Examples.GeneralInformation;
import org.firstinspires.ftc.teamcode.MainParts.Examples.Robot.Config.MainFile;
import org.firstinspires.ftc.teamcode.MainParts.Examples.Robot.RobotParts.Odometry.Parts.MathUtils.Position2D;
import org.firstinspires.ftc.teamcode.MainParts.Examples.Robot.RobotParts.Odometry.Parts.MathUtils.Position3D;
import org.firstinspires.ftc.teamcode.MainParts.Examples.Robot.RobotParts.Odometry.Parts.MathUtils.Vector2;
import org.firstinspires.ftc.teamcode.MainParts.Modules.Extenders.UpdatableCollector;
import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;


import java.util.concurrent.TimeUnit;

public class CameraClass extends UpdatableCollector {
    private WebcamName webcamName;
    private Position cameraPosition;
    private YawPitchRollAngles cameraOrientation;
    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;
    public GeneralLogic generalLogic;
    public RandomizeStatus randomizeStatus;
    public GeneralInformation generalInformation;
    public ElapsedTime updateTime;
    public CameraClass(){
        super(false);
        generalInformation = MainFile.generalInformation;
        try {
            webcamName = MainFile.op.hardwareMap.get(WebcamName.class, controlHubDevices.webcam1);
            cameraPosition = new Position(DistanceUnit.CM,-19, 0,0, 0);//Позиция камеры относительно координат робота
// СМЕЩЕНИЕ КАМЕРЫ НА 19 СМ К СБОРЩИКУ
            cameraOrientation = new YawPitchRollAngles(AngleUnit.RADIANS, Math.toRadians(-90) , Math.toRadians(-85), Math.toRadians(0), 0);

            aprilTagProcessor = new AprilTagProcessor.Builder()
                    .setDrawAxes(false)
                    .setDrawCubeProjection(false)
                    .setDrawTagID(true)
                    .setDrawTagOutline(false)
                    .setLensIntrinsics(705.693,705.693,322.799, 239.459)
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
                    .enableLiveView(true)
                    .build();
        }catch (Exception e){
            isInitialized = false;
        }

        generalLogic = GeneralLogic.Check_camera_state;

        randomizeStatus = RandomizeStatus.UnDetected;
        absoluteRawPos3D = new Position3D();

        updateTime = new ElapsedTime();
        sayCreated();
    }
    public Position3D absoluteRawPos3D;
    private ExposureControl exposure;
    private GainControl gain;
    private int index;

    public enum RandomizeStatus{
        Detected,
        UnDetected
    }
    public enum GeneralLogic{
        Check_camera_state,
        Processing,
        Stop
    }
    protected double cameraPitch;
    protected double cameraRoll;
    protected double cameraYaw;
    public int id;
    private boolean isCameraCoverUp = false;
    public double decisionMargin = 0;

    public void coverUpCamera(){
        isCameraCoverUp = true;
    }
    public void openUpCamera(){
        isCameraCoverUp = false;
    }
    public Position2D robotPos = new Position2D();
    public int[] motif = new int[3];
    public double head1, head2;
    double head;
    double searchingHead;
    Vector2 globalVector = new Vector2();

    public double camToTagX;
    public double camToTagY;
    public double range;
    public double bearing;
    public double yaw;
    @Override
    protected void updateExt() {
        switch (generalLogic){
            case Check_camera_state:
                if (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {

                    exposure = visionPortal.getCameraControl(ExposureControl.class);
                    exposure.setMode(ExposureControl.Mode.Manual);//Если камера не поддерживает настройку экспозиции
                    exposure.setExposure(5, TimeUnit.MILLISECONDS);//Экспозиция

                    gain = visionPortal.getCameraControl(GainControl.class);

                    //утром 130 - вечером 170
                    gain.setGain(130);//яркость

                    aprilTagProcessor.setDecimation(2.0f);

                    generalLogic = GeneralLogic.Processing;
                }
                break;
            case Processing:
                //Фиксируем один раз
                AprilTagProcessor aprilTagDetection = aprilTagProcessor;
                if (!aprilTagDetection.getDetections().isEmpty() && !isCameraCoverUp)
                {
                    index = index % aprilTagDetection.getDetections().size();

                    AprilTagDetection detection = aprilTagDetection.getDetections().get(index);

                    index++;

                    id = detection.id;

                    setRandomizedArtifactFromId(id);

                    if (id == 20 || id == 24){
                        decisionMargin = 100;
                        range = detection.ftcPose.range;      // расстояние до тега (см)
                        bearing = detection.ftcPose.bearing;  // горизонтальный угол (рад)
                        yaw = detection.ftcPose.yaw;          // угол тега относительно камеры
                        double elev = detection.ftcPose.elevation + Math.toRadians(0);

                        camToTagX = range * Math.cos(elev) * Math.cos(bearing);
                        camToTagY = range * Math.cos(elev) * Math.sin(bearing);

                        double camOffsetX = 0;
                        double camOffsetY = 19;

                        camToTagX += camOffsetX;
                        camToTagY += camOffsetY;
                    }
                    else{
                        decisionMargin = 0;
                    }
                }
                else
                {
                    decisionMargin = 0;
                    setRandomizedArtifactFromId(0);
                    index = 0;
                }
                break;

            case Stop:
                visionPortal.stopStreaming();
                break;
        }
}

    public void setRandomizedArtifactFromId(int id) {
        int green = 1;
        int purple = 2;

        if ((id != 21 || id != 22 || id != 23) && randomizeStatus == RandomizeStatus.UnDetected && motif[0] == 0){
            int min = 21;
            int max = 23;
            id = (int)(Math.random() * (max - min + 1)) + min;

            if(id == 21){
                motif = new int[] {green, purple, purple};
            }
            if(id == 22){
                motif = new int[] {purple, green, purple};
            }
            if(id == 23){
                motif = new int[] {purple, purple, green};
            }
        }else {
            if(id == 21){
                motif = new int[] {green, purple, purple};
            }
            if(id == 22){
                motif = new int[] {purple, green, purple};
            }
            if(id == 23){
                motif = new int[] {purple, purple, green};
            }
            randomizeStatus = RandomizeStatus.Detected;
        }
    }

    @Override
    public void showDataExt() {
        telemetry.addData("Ve", "%s %s",globalVector.x, globalVector.y);
        telemetry.addData("Searching", searchingHead * RAD);
        telemetry.addData("headinsg", "%s ftcYaw %s bear %s",head * RAD,yaw

                * RAD, bearing *RAD );
        telemetry.addData("robotL", robotPos.toVector().length());
        telemetry.addData("Range", range);
        telemetry.addData("Headings", "Y %s R %s P %s", cameraYaw * RAD, cameraRoll * RAD, cameraPitch * RAD);
        telemetry.addData("General logic", generalLogic.toString());
        telemetry.addData("Randomize status", randomizeStatus.toString());
        telemetry.addData("motif", "%s %s %s",motif[0], motif[1], motif[2]);
        telemetry.addData("des", decisionMargin);
    }
}
