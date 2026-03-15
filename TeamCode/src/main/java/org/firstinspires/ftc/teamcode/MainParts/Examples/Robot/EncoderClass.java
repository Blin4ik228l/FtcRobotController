package org.firstinspires.ftc.teamcode.MainParts.Examples.Robot;

import org.firstinspires.ftc.teamcode.MainParts.Examples.Robot.RobotParts.Odometry.OdometryBuffer;
import org.firstinspires.ftc.teamcode.MainParts.Examples.Robot.RobotParts.Odometry.OdometryData;
import org.firstinspires.ftc.teamcode.MainParts.Modules.Extenders.UpdatableCollector;

public abstract class EncoderClass extends UpdatableCollector {
    public OdometryBuffer odometryBuffer;
    public SelfMath selfMath;
    public EncoderClass() {
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
