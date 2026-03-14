package org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.HoodedShoter.Modules;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Players.Enums.Units;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.Config.MainFile;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.InnerMath;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.Odometry.OdometryBuffer;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.Odometry.OdometryData;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.Odometry.Parts.MathUtils.Position2D;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.Wrappers.Examples.MotorWrapper;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Modules.Extenders.ExecutableCollector;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Modules.Extenders.UpdatableCollector;

public class TurretMotor extends ExecutableCollector {
    public String turretMotor = controlHubDevices.getMotor(0);
    public TurretOdometry turretOdometry;
    public boolean isNeedBack;

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
    public class TurretOdometry extends UpdatableCollector {
        public TurretOdometry(MainFile mainFile, MotorWrapper.InnerCollector motors) {
            super(mainFile);
            motorsCollector = motors;

            turretBuffer = new OdometryBuffer();
            selfMath = new SelfMath();

            sayCreated();
        }
        public OdometryBuffer turretBuffer;
        public double localHead = 0;
        public boolean wasGreaterThenPI;
        private SelfMath selfMath;

        @Override
        protected void updateExt() {
            selfMath.calculateAll();
        }

        @Override
        protected void showDataExt() {
            telemetry.addData("TuretData", "head  %.2f vel %.2f", localHead * RAD, turretBuffer.read().getHeadVel() * RAD);
            telemetry.addData("NeedBack", isNeedBack);
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
                curMotorPos = getCurentPos(motorsCollector.get(turretMotor), Units.Rad);
                deltaPos = lastMotorPos - curMotorPos;
                lastMotorPos = curMotorPos;

                deltaHead = -deltaPos;

                curTime = runTime.milliseconds();
                deltaTime = curTime - lastTime;
                lastTime = curTime;

                double headVel = deltaHead / (deltaTime / 1000);

                double headVel2 = getCurrentVelocity(motorsCollector.get(turretMotor), Units.Rad);

                filteredTurretVelocity = filtr * headVel2 + (1 - filtr) * filteredTurretVelocity;

                localHead += deltaHead;

                rawData.setPosition(new Position2D(0,0, deltaHead));
                rawData.setHeadVel(filteredTurretVelocity);

                turretBuffer.beginWrite().set(rawData);
                turretBuffer.endWrite2();
            }
        }
    }

}
