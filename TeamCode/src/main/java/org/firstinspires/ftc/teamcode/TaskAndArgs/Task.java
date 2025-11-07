package org.firstinspires.ftc.teamcode.TaskAndArgs;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Modules.Handlers.DriveHandler;
import org.firstinspires.ftc.teamcode.Modules.Handlers.Handler;
import org.firstinspires.ftc.teamcode.Modules.Handlers.TelescopeHandler;
import org.firstinspires.ftc.teamcode.Robot.RobotParts.DrivetrainParts.Odometry.Parts.MathUtils.Position;
import org.firstinspires.ftc.teamcode.Robot.RobotClass;

public class Task {
    public Task(double targetX, double targetY, double targetHeading, double targetSpeed, RobotClass.MecanumDrivetrain driveTrain, Telemetry telemetry, int queuePlace){
        Position targetPosition = new Position(targetX, targetY, targetHeading);
        Args.DriveArgs driveArgs = new Args.DriveArgs(targetPosition, targetSpeed);
        DriveHandler driveHandler = new DriveHandler(driveTrain, telemetry);

        driveHandler.setArgs(driveArgs);
        handler = driveHandler;

        this.queuePlace = queuePlace;
    }

    public Task(double targetHeight, double targetPower, RobotClass.Collector teleSkope, Telemetry telemetry, int queuePlace){
        Args.LiftArgs liftArgs = new Args.LiftArgs(targetHeight, targetPower);
        TelescopeHandler teleskopeHandler = new TelescopeHandler(teleSkope, telemetry);

        teleskopeHandler.setArgs(liftArgs);
        handler = teleskopeHandler;

        this.queuePlace = queuePlace;
    }

    public Task(String targetName, double targetPos, RobotClass.Collector teleSkope, Telemetry telemetry, int queuePlace){
        Args.ServoArgs servoArgs = new Args.ServoArgs(targetName, targetPos);
        TelescopeHandler telescopeHandler = new TelescopeHandler(teleSkope, telemetry);

        telescopeHandler.setArgs(servoArgs);
        handler = telescopeHandler;

        this.queuePlace = queuePlace;
    }

    public Task(double targetHeight, double targetPower, String targetName, double targetPos, RobotClass.Collector teleSkope, Telemetry telemetry, int queuePlace){
        Args.LiftArgs liftArgs = new Args.LiftArgs(targetHeight, targetPower);
        Args.ServoArgs servoArgs = new Args.ServoArgs(targetName, targetPos);
        TelescopeHandler telescopeHandler = new TelescopeHandler(teleSkope, telemetry);

        telescopeHandler.setArgs(liftArgs);
        telescopeHandler.setArgs(servoArgs);

        handler = telescopeHandler;

        this.queuePlace = queuePlace;
    }
    public boolean isDone = false;
    public Handler handler;
    public int queuePlace;
    public void startDoing(){
        handler.execute();
        isDone = handler.isDone;
    }
}
