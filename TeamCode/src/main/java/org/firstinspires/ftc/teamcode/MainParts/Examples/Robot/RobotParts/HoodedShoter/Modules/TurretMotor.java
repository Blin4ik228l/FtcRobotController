package org.firstinspires.ftc.teamcode.MainParts.Examples.Robot.RobotParts.HoodedShoter.Modules;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MainParts.Examples.Players.Enums.Units;
import org.firstinspires.ftc.teamcode.MainParts.Examples.Robot.RobotParts.Odometry.OdometryBuffer;
import org.firstinspires.ftc.teamcode.MainParts.Examples.Robot.RobotParts.Odometry.OdometryData;
import org.firstinspires.ftc.teamcode.MainParts.Examples.Robot.RobotParts.Odometry.Parts.MathUtils.Position2D;
import org.firstinspires.ftc.teamcode.MainParts.Examples.Wrappers.Examples.MotorWrapper;
import org.firstinspires.ftc.teamcode.MainParts.Modules.Extenders.ExecutableCollector;
import org.firstinspires.ftc.teamcode.MainParts.Modules.Extenders.UpdatableCollector;

public class TurretMotor extends ExecutableCollector {
    public OdometryBuffer turretBuffer;
    public double localHead = 0;
    public ClassMath classMath;
    public TurretMotor() {
        super(false);
        createMotorWrapperUtils();
        motorsCollector.add(motorBuilder.initialize(turretMotor).setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER).setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER).setDirection(DcMotorSimple.Direction.FORWARD).setBehavior(DcMotor.ZeroPowerBehavior.FLOAT)
                .setFields(13.0, 1.0, 384.5, 2.0, 5.19).get());

        classMath = new ClassMath(motorsCollector);
        sayCreated();
    }
    public String turretMotor = controlHubDevices.getMotor(0);
    @Override
    protected void executeExt(Double... args) {
        motorsCollector.get(turretMotor).execute(args);
    }

    @Override
    protected void showDataExt() {
        telemetry.addData("ticks", motorsCollector.get(turretMotor).getMotorConfigurationType().getTicksPerRev());
        motorsCollector.showData();
        classMath.showData();
    }
    public class ClassMath extends UpdatableCollector {
        private OdometryData rawData;
        private double fltrdHeadVel;
        private double headAccel;
        private double deltaHead;
        private double curVel, deltaVel, lastVel;
        private double lastMotorPos, curMotorPos, deltaPos;
        private double curTime, deltaTime, lastTime;
        private final ElapsedTime runTime;
        public ClassMath(MotorWrapper.InnerCollector motors) {
            super(false);
            motorsCollector = motors;
            fltrdHeadVel = 0.0;

            turretBuffer = new OdometryBuffer();
            rawData = new OdometryData();

            runTime = new ElapsedTime();

            sayCreated();
        }

        @Override
        protected void updateExt() {
            double filtr = 1;
            //Тиков на оборот мотора
            curMotorPos = motorsCollector.get(turretMotor).getCurPos(Units.Rad);
            deltaPos = lastMotorPos - curMotorPos;
            lastMotorPos = curMotorPos;

            deltaHead = deltaPos;

            localHead += deltaHead;

            double headVel = -motorsCollector.get(turretMotor).getCurVel(Units.Rad);

            fltrdHeadVel = filtr * headVel + (1 - filtr) * fltrdHeadVel;

            curTime = runTime.milliseconds();
            deltaTime = curTime - lastTime;
            lastTime = curTime;

            curVel = fltrdHeadVel;
            deltaVel = curVel - lastVel;
            lastVel = curVel;

            if(deltaTime == 0) headAccel = 0;
            else headAccel = (deltaVel / deltaTime);

            rawData.setPosition(new Position2D(0,0,deltaHead));
            rawData.setHeadVel(fltrdHeadVel);
            rawData.setHeadAccel(headAccel);

            turretBuffer.beginWrite().set(rawData);
            turretBuffer.endWrite2();
        }

        @Override
        protected void showDataExt() {
            telemetry.addData("TuretData", "head  %.2f vel %.2f", localHead * RAD, turretBuffer.read().getHeadVel() * RAD);
        }
    }
}
