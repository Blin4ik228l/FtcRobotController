package org.firstinspires.ftc.teamcode.Robot;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Modules.Module;
import org.firstinspires.ftc.teamcode.Robot.RobotParts.CollectorParts.MotorsOnCollector;

public class MotorsControllerClass extends Module {
    public final MotorsOnCollector motors;
    public double inTakePower;
    public double radianSpeed;

    public MotorsControllerClass(MotorsOnCollector motorsOnCollector, Telemetry telemetry){
        super(telemetry);
        motors = motorsOnCollector;
    }
    public void execute(){
        motors.turnOnInTake(inTakePower);
        motors.setSpeedOnFlyWheel(radianSpeed);
    }
}
