package org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.HoodedShoter.Modules;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Players.PL0.Units;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.Config.MainFile;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.InnerMath;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.Odometry.OdometryBuffer;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.Odometry.OdometryData;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.Odometry.Parts.MathUtils.Position2D;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.VoltageSensorClass;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.Wrappers.Examples.MotorWrapper;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Modules.Extenders.ExecutingModule;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Modules.Extenders.Extenders2.UpdatableModule;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Modules.Extenders.UpdatingModule;

public class TurretMotor extends ExecutingModule {
    public String turretMotor = controlHubDevices.getMotor(0);
    public TurretOdometry turretOdometry;
    public boolean isInterrupted;

    public TurretMotor(MainFile mainFile) {
        super(mainFile);

        createMotorWrapperUtils();
        motorsCollector.add(motorBuilder.initialize(mainFile, turretMotor).setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER).setDirection(DcMotorSimple.Direction.FORWARD).setBehavior(DcMotor.ZeroPowerBehavior.FLOAT)
                .setFields(12.5, 1.0).get());

        turretOdometry = new TurretOdometry(mainFile, motorsCollector);
        sayCreated();
    }

    @Override
    protected void executeExt(Double... args) {
        motorsCollector.get(turretMotor).execute(args);

        turretOdometry.update();
    }

    @Override
    protected void showDataExt() {
        motorsCollector.showData();
        turretOdometry.showData();
    }
    public class TurretOdometry extends UpdatingModule {
        public TurretOdometry(MainFile mainFile, MotorWrapper.InnerCollector motors) {
            super(mainFile);
            motorsCollector = motors;

            turretBuffer = new OdometryBuffer();
            selfMath = new SelfMath();

            sayCreated();
        }
        public OdometryBuffer turretBuffer;
        public double localHead;
        private SelfMath selfMath;

        @Override
        protected void updateExt() {
            selfMath.calculateAll();
        }

        @Override
        protected void showDataExt() {
            telemetry.addData("TuretData", "head  %s vel %s", localHead , turretBuffer.read().getHeadVel() * RAD);
        }

        public class SelfMath extends InnerMath {
            private OdometryData rawData;
            private double filteredTurretVelocity;
            private double deltaHead;
            private double lastMotorPos, curMotorPos, deltaPos;
            private ElapsedTime runTime;
            public SelfMath(){
                setRadius(14.8).setDRIVE_GEAR_REDUCTION(1).setCOUNTS_PER_ENCODER_REV(200);

                rawData = new OdometryData();

                runTime = new ElapsedTime();
            }
            private double curTime, deltaTime, lastTime;
            public void calculateAll(){
                double filtr = 1;
                //Тиков на оборот мотора
                double outPutResolution = 288;

                curMotorPos = getCurentPos(motorsCollector.get(turretMotor), Units.Ticks);
                deltaPos = lastMotorPos - curMotorPos;
                lastMotorPos = curMotorPos;

                deltaHead = -(deltaPos / outPutResolution) * 2 * Math.PI;

                curTime = runTime.milliseconds();
                deltaTime = curTime - lastTime;
                lastTime = curTime;

                double headVel = deltaHead / (deltaTime / 1000);

                double headVel2 = getCurrentVelocity(motorsCollector.get(turretMotor), Units.Rad);

                filteredTurretVelocity = filtr * headVel + (1 - filtr) * filteredTurretVelocity;

                if(Math.abs(curMotorPos) > outPutResolution) isInterrupted = true;

                localHead += deltaHead;

                rawData.setPosition(new Position2D(0,0, deltaHead));
                rawData.setHeadVel(filteredTurretVelocity);

                turretBuffer.beginWrite().set(rawData);
                turretBuffer.endWrite2();
            }
        }
    }

}
