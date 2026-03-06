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
        motorsCollector.add(motorBuilder.initialize(mainFile, turretMotor).setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER).setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER).setDirection(DcMotorSimple.Direction.REVERSE).setBehavior(DcMotor.ZeroPowerBehavior.FLOAT)
                .setFields(13.0, 1.0).get());

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
        public double localHead = Math.toRadians(180);
        private SelfMath selfMath;

        @Override
        protected void updateExt() {
            selfMath.calculateAll();
        }

        @Override
        protected void showDataExt() {
            telemetry.addData("TuretData", "head  %s vel %s", localHead * RAD, turretBuffer.read().getHeadVel() * RAD);
        }

        public class SelfMath extends InnerMath {
            private OdometryData rawData;
            private double filteredTurretVelocity;
            private double deltaHead;
            private double lastMotorPos, curMotorPos, deltaPos;
            private ElapsedTime runTime;

            public SelfMath(){
                setRadius(2).setDRIVE_GEAR_REDUCTION(5.19).setCOUNTS_PER_ENCODER_REV(384.5).calculateCountsPerCm();

                rawData = new OdometryData();

                runTime = new ElapsedTime();
            }
            private double curTime, deltaTime, lastTime;
            public void calculateAll(){
                double filtr = 1;
                //Тиков на оборот мотора
                double outPutResolution = 384.5 * 5.19;

                curMotorPos = getCurentPos(motorsCollector.get(turretMotor), Units.Ticks);
                deltaPos = lastMotorPos - curMotorPos;
                lastMotorPos = curMotorPos;

                deltaHead = getCurentPos(motorsCollector.get(turretMotor), Units.Rad);;

                curTime = runTime.milliseconds();
                deltaTime = curTime - lastTime;
                lastTime = curTime;

                double headVel = deltaHead / (deltaTime / 1000);

                double headVel2 = getCurrentVelocity(motorsCollector.get(turretMotor), Units.Rad);

                filteredTurretVelocity = filtr * headVel2 + (1 - filtr) * filteredTurretVelocity;

                Position2D normPos = new Position2D(0,0,localHead);

                if(normPos.getHeading() - 2 * Math.PI >= 2 * Math.PI) isInterrupted = true;

                localHead += deltaHead;

                rawData.setPosition(new Position2D(0,0, deltaHead));
                rawData.setHeadVel(filteredTurretVelocity);

                turretBuffer.beginWrite().set(rawData);
                turretBuffer.endWrite2();
            }
        }
    }

}
