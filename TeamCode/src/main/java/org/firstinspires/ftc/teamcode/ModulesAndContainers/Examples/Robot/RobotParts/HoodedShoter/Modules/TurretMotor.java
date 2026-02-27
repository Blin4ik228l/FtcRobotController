package org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.HoodedShoter.Modules;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Players.PL0.Units;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.Odometry.OdometryBuffer;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.Odometry.OdometryData;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.Odometry.Parts.MathUtils.Position2D;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.VoltageSensorClass;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Modules.Extenders.MotorModule;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Modules.Extenders.UpdatableModule;

public class TurretMotor extends MotorModule {
    public String turretMotor = controlHubDevices.getMotor(0);
    public TurretOdometry turretOdometry;

    public boolean isInterrupted;

    public TurretMotor(OpMode op, VoltageSensorClass voltageSensorClass) {
        super(op);
        motorWrapper = motorBuilder.initialize(op, turretMotor).setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER).setDirection(DcMotorSimple.Direction.FORWARD).setBehavior(DcMotor.ZeroPowerBehavior.FLOAT)
                .setFields(voltageSensorClass, 12.5, 1).get();

        turretOdometry = new TurretOdometry(op);
        sayInited();
    }
    public void setPower(double power){
        motorWrapper.setPower(power);
    }
    @Override
    public void showData() {
        sayModuleName();
        motorsWrapper.showData();
        turretOdometry.showData();
    }
    public class TurretOdometry extends UpdatableModule {
        public TurretOdometry(OpMode op) {
            super(op);
            innerMath
                    .setRadius(14.8)
                    .setDRIVE_GEAR_REDUCTION(1)
                    .setCOUNTS_PER_ENCODER_REV(200);

            turretBuffer = new OdometryBuffer();
            selfMath = new SelfMath();

            sayInited();
        }
        public OdometryBuffer turretBuffer;
        public boolean switcher = false;
        public double localHead;
        private SelfMath selfMath;

        @Override
        public void update() {
            selfMath.calculateAll();
        }

        @Override
        public void showData() {
            if (!isInitialized){telemetry.addLine("Need motor to be init");}
            else telemetry.addData("TuretData", "head  %s vel %s", localHead , turretBuffer.read().getHeadVel() * RAD);
        }

        public class SelfMath{
            private OdometryData rawData;
            private double filteredTurretVelocity;
            private double deltaHead;
            private double lastMotorPos, curMotorPos, deltaPos;
            private ElapsedTime runTime;
            public SelfMath(){
                rawData = new OdometryData();

                runTime = new ElapsedTime();
            }
            private double curTime, deltaTime, lastTime;
            public void calculateAll(){
                double filtr = 1;
                //Тиков на оборот мотора
                double outPutResolution = 288;

                curMotorPos = innerMath.getCurentPos(turretMotor, Units.Ticks);

                deltaPos = lastMotorPos - curMotorPos;
                lastMotorPos = curMotorPos;

                deltaHead = -(deltaPos / outPutResolution) * 2 * Math.PI;

                curTime = runTime.milliseconds();
                deltaTime = curTime - lastTime;
                lastTime = curTime;

                double headVel = deltaHead / (deltaTime / 1000);

                double headVel2 = innerMath.getCurrentVelocity(turretMotor, Units.Rad);

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
