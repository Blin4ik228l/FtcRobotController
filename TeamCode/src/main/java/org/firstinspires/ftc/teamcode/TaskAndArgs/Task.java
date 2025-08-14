package org.firstinspires.ftc.teamcode.TaskAndArgs;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Modules.Handlers.DriveHandler;
import org.firstinspires.ftc.teamcode.Modules.Handlers.Handler;
import org.firstinspires.ftc.teamcode.Modules.Handlers.TeleSkopeHandler;
import org.firstinspires.ftc.teamcode.Modules.Players.Pl1.MecanumDriveTrain.MathUtils.Position;
import org.firstinspires.ftc.teamcode.Modules.Players.Pl1.MecanumDriveTrain.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.Modules.Players.Pl2.TeleSkope;

public class Task {
    public Task(double targetX, double targetY, double targetHeading, double targetSpeed, MecanumDriveTrain driveTrain, Telemetry telemetry, int queuePlace){
        Position targetPosition = new Position(targetX, targetY, targetHeading);
        Args.DriveArgs driveArgs = new Args.DriveArgs(targetPosition, targetSpeed);
        DriveHandler driveHandler = new DriveHandler(driveTrain, telemetry);

        driveHandler.setArgs(driveArgs);
        handler = driveHandler;

        this.queuePlace = queuePlace;
    }

    public Task(double targetHeight, double targetPower, TeleSkope teleSkope, Telemetry telemetry, int queuePlace){
        Args.LiftArgs liftArgs = new Args.LiftArgs(targetHeight, targetPower);
        TeleSkopeHandler teleskopeHandler = new TeleSkopeHandler(teleSkope, telemetry);

        teleskopeHandler.setArgs(liftArgs);
        handler = teleskopeHandler;

        this.queuePlace = queuePlace;
    }

    public Task( String targetName, double targetPos, TeleSkope teleSkope, Telemetry telemetry, int queuePlace){
        Args.ServoArgs servoArgs = new Args.ServoArgs(targetName, targetPos);
        TeleSkopeHandler teleSkopeHandler = new TeleSkopeHandler(teleSkope, telemetry);

        teleSkopeHandler.setArgs(servoArgs);
        handler = teleSkopeHandler;

        this.queuePlace = queuePlace;
    }

    public Task(double targetHeight, double targetPower, String targetName, double targetPos, TeleSkope teleSkope, Telemetry telemetry, int queuePlace){
        Args.LiftArgs liftArgs = new Args.LiftArgs(targetHeight, targetPower);
        Args.ServoArgs servoArgs = new Args.ServoArgs(targetName, targetPos);
        TeleSkopeHandler teleSkopeHandler = new TeleSkopeHandler(teleSkope, telemetry);

        teleSkopeHandler.setArgs(liftArgs);
        teleSkopeHandler.setArgs(servoArgs);

        handler = teleSkopeHandler;

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
