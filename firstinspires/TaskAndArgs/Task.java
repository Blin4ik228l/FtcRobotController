package org.firstinspires.TaskAndArgs;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Modules.Extenders.ExecutableModule;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.HoodedShoter.Collector;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.DriveTrain.DrivetrainParts.Odometry.Parts.MathUtils.Position2D;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.DriveTrain.MecanumDrivetrain;

public class Task {
    public Task(double targetX, double targetY, double targetHeading, double targetSpeed, MecanumDrivetrain driveTrain, Telemetry telemetry, int queuePlace){
        Position2D targetPosition2D = new Position2D(targetX, targetY, targetHeading);
        Args.DriveArgs driveArgs = new Args.DriveArgs(targetPosition2D, targetSpeed);


        this.queuePlace = queuePlace;
    }

    public Task(double targetHeight, double targetPower, Collector teleSkope, Telemetry telemetry, int queuePlace){
        Args.LiftArgs liftArgs = new Args.LiftArgs(targetHeight, targetPower);

        this.queuePlace = queuePlace;
    }

    public Task(String targetName, double targetPos, Collector teleSkope, Telemetry telemetry, int queuePlace){
        Args.ServoArgs servoArgs = new Args.ServoArgs(targetName, targetPos);

        this.queuePlace = queuePlace;
    }

    public Task(double targetHeight, double targetPower, String targetName, double targetPos, Collector teleSkope, Telemetry telemetry, int queuePlace){
        Args.LiftArgs liftArgs = new Args.LiftArgs(targetHeight, targetPower);
        Args.ServoArgs servoArgs = new Args.ServoArgs(targetName, targetPos);

        this.queuePlace = queuePlace;
    }
    public boolean isDone = false;
    public ExecutableModule executableModule;
    public int queuePlace;
    public void startDoing(){
        executableModule.execute();
    }
}
