package org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.HoodedShoter.Modules;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.Odometry.OdometryBuffer;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.Odometry.OdometryData;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.Odometry.Parts.MathUtils.Position2D;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.Wrappers.MotorWrapper;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Modules.Extenders.MotorModule;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Modules.Extenders.UpdatableModule;

public class TurretMotor extends MotorModule {
    public TurretOdometry turretOdometry;
    public boolean isInterrupted;
    public String deviceName = "m1";
    public TurretMotor(OpMode op) {
        super(op);
        motorsWrapper.add(op, dcMotorEx.initialize(op, deviceName).setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER).setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER).setDirection(DcMotorSimple.Direction.FORWARD).setBehavior(DcMotor.ZeroPowerBehavior.FLOAT));

        turretOdometry = new TurretOdometry(op, isInitialized);
        sayInited();
    }
    public void setPower(double power){
        if(!isInitialized) return;
        double currentVoltage = voltageSensorClass.getCurVoltage();
        if (currentVoltage <= 0) currentVoltage = 11.5;

        double voltageMultiplier = 12.5 / currentVoltage;

        power *= voltageMultiplier;

        power = Range.clip(power, -1.0, 1.0);
        motorsWrapper.get(deviceName).setPower(power);
//        turretOdometry.update();
    }
    public MotorWrapper getWrapper(){
        return motorsWrapper.get(deviceName);
    }
    @Override
    public void showData() {
        sayModuleName();
        motorsWrapper.showData();
    }
    public class TurretOdometry extends UpdatableModule {
        public TurretOdometry(OpMode op, boolean isInit) {
            super(op);
            turretBuffer = new OdometryBuffer();
            selfMath = new SelfMath();

            this.isInitialized = isInit;
            sayInited();
        }
        public OdometryBuffer turretBuffer;
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

                curMotorPos = motorsWrapper.get(deviceName).getMotor().getCurrentPosition();
                deltaPos = lastMotorPos - curMotorPos;
                lastMotorPos = curMotorPos;

                deltaHead = -(deltaPos / outPutResolution) * 2 * Math.PI;

                curTime = runTime.milliseconds();
                deltaTime = curTime - lastTime;
                lastTime = curTime;

                double headVel = deltaHead / (deltaTime / 1000);

                double headVel2 = motorsWrapper.get(deviceName).getMotorEx().getVelocity(AngleUnit.RADIANS);

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
