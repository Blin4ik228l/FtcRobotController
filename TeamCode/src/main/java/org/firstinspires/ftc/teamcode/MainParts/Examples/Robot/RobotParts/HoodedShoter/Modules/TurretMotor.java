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
    public String turretMotor = controlHubDevices.getMotor(0);
    public EncodersClass encodersClass;
    public boolean isNeedBack;
    public TurretMotor() {
        super(false);
        createMotorWrapperUtils();
        motorsCollector.add(motorBuilder.initialize(turretMotor).setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER).setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER).setDirection(DcMotorSimple.Direction.REVERSE).setBehavior(DcMotor.ZeroPowerBehavior.FLOAT)
                .setFields(13.0, 1.0, 0.0, 2.0, 5.19).get());

        encodersClass = new EncodersClass(motorsCollector);
        sayCreated();
    }

    @Override
    protected void executeExt(Double... args) {
        motorsCollector.get(turretMotor).execute(args);

    }

    @Override
    protected void showDataExt() {
        motorsCollector.showData();
        encodersClass.showData();
    }
    public class EncodersClass extends UpdatableCollector {
        private OdometryData rawData;
        private double filteredTurretVelocity;
        private double deltaHead;
        private double lastMotorPos, curMotorPos, deltaPos;
        private ElapsedTime runTime;
        public EncodersClass(MotorWrapper.InnerCollector motors) {
            super(false);
            motorsCollector = motors;

            turretBuffer = new OdometryBuffer();
            rawData = new OdometryData();

            runTime = new ElapsedTime();

            sayCreated();
        }
        public OdometryBuffer turretBuffer;
        public double localHead = 0;
        public boolean wasGreaterThenPI;
        private double curTime, deltaTime, lastTime;

        @Override
        protected void updateExt() {
            double filtr = 1;
            //Тиков на оборот мотора
            curMotorPos = motorsCollector.get(turretMotor).getCurPos(Units.Rad);
            deltaPos = lastMotorPos - curMotorPos;
            lastMotorPos = curMotorPos;

            deltaHead = -deltaPos;

            curTime = runTime.milliseconds();
            deltaTime = curTime - lastTime;
            lastTime = curTime;

            double headVel = deltaHead / (deltaTime / 1000);

            double headVel2 = motorsCollector.get(turretMotor).getCurVel(Units.Rad);

            filteredTurretVelocity = filtr * headVel2 + (1 - filtr) * filteredTurretVelocity;

            localHead += deltaHead;

            rawData.setPosition(new Position2D(0,0, deltaHead));
            rawData.setHeadVel(filteredTurretVelocity);

            turretBuffer.beginWrite().set(rawData);
            turretBuffer.endWrite2();
        }

        @Override
        protected void showDataExt() {
            telemetry.addData("TuretData", "head  %.2f vel %.2f", localHead * RAD, turretBuffer.read().getHeadVel() * RAD);
            telemetry.addData("NeedBack", isNeedBack);
        }
    }

}
