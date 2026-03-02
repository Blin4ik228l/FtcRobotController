package org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot;

import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.Config.MainFile;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.DriveTrain.DrivetrainParts.DrivetrainMotors;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.HoodedShoter.Modules.TurretMotor;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.Odometry.OdometryBuffer;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.Odometry.OdometryData;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Modules.Extenders.UpdatingModule;

public abstract class EncoderClass extends UpdatingModule {
    public OdometryBuffer odometryBuffer;
    public SelfMath selfMath;
    public EncoderClass(MainFile mainFile) {
        super(mainFile);
        odometryBuffer = new OdometryBuffer();
    }

    public abstract class SelfMath extends InnerMath {
        public OdometryData rawData;
        public SelfMath(){
            rawData = new OdometryData();
        }
        public void calculateAll(){
            calcPos();
            calcSpeed();
            odometryBuffer.beginWrite().set(rawData);
            odometryBuffer.endWrite2();
        };

        public abstract void calcSpeed();
        public abstract void calcPos();
    }
}
