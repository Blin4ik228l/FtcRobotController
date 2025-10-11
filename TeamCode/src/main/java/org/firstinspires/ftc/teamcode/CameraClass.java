package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.CameraControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Modules.Module;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;

public class CameraClass {
    public AprilTagProcessor aprilTagProcessor;
    public VisionPortal visionPortal;

    public Telemetry telemetry;
    public WebcamName webcamName;
    public CameraClass(OpMode op, Telemetry telemetry) {
        this.telemetry = telemetry;

        webcamName = op.hardwareMap.get(WebcamName.class, "Webcam 1");

        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setOutputUnits(DistanceUnit.CM, AngleUnit.DEGREES)
                .setTagLibrary(AprilTagGameDatabase.getDecodeTagLibrary())
                .build();

        visionPortal = new VisionPortal.Builder()
                .addProcessor(aprilTagProcessor)
                .setCamera(webcamName)
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .build();
    }

    public void execute() {
        while (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING){}

        ExposureControl exposure = visionPortal.getCameraControl(ExposureControl.class);
        exposure.setMode(ExposureControl.Mode.Manual);//Если камера не поддерживает настройку экспозиции
        exposure.setExposure(5, TimeUnit.MILLISECONDS);//Экспозиция

        GainControl gain = visionPortal.getCameraControl(GainControl.class);
        gain.setGain(255);//яркость



//        CameraControl cameraControl = visionPortal.getCameraControl()
        aprilTagProcessor.setDecimation(3);
        if (!aprilTagProcessor.getDetections().isEmpty()) {

            AprilTagDetection aprilTagDetection = aprilTagProcessor.getDetections().get(0);


            telemetry.addLine("Tag detection")
                    .addData("\nX pose", aprilTagDetection.ftcPose.x)
                    .addData("\nY pose", aprilTagDetection.ftcPose.y)
                    .addData("\nZ pose", aprilTagDetection.ftcPose.z)
                    .addData("\nroll", aprilTagDetection.ftcPose.roll)
                    .addData("\npitch", aprilTagDetection.ftcPose.pitch)
                    .addData("\nyaw", aprilTagDetection.ftcPose.yaw)
                    .addData("\nroll", aprilTagDetection.ftcPose.bearing)
                    .addData("\npitch", aprilTagDetection.ftcPose.elevation)
                    .addData("\nyaw", aprilTagDetection.ftcPose.range);
            telemetry.addLine();


        }
    }
}
