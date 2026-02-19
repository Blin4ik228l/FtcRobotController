package org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.HoodedShoter.Modules;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.Wrappers.MotorWrapper;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.CameraClass;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Modules.Extenders.MotorModule;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Modules.Extenders.UpdatableModule;

public class TurretMotor extends MotorModule {
    public TurretOdometry turretOdometry;
    public boolean isInterrupted;
    public TurretMotor(OpMode op) {
        super(op);
        motorsWrapper.add(op, dcMotorEx.initialize("turretMotor").setBehavior(DcMotor.ZeroPowerBehavior.FLOAT).setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER).get());

        turretOdometry = new TurretOdometry(op);
    }
    public void setPower(double headPow){
        motorsWrapper.get("turretMotor").setPower(headPow);
        turretOdometry.update();
    }
    public MotorWrapper getWrapper(){
        return motorsWrapper.get("turretMotor");
    }
    @Override
    public void showData() {

    }
    public class TurretOdometry extends UpdatableModule {
        public TurretOdometry(OpMode op) {
            super(op);
            selfMath = new SelfMath();
        }
        public double turrelLocalHead, headVel;
        private SelfMath selfMath;

        @Override
        public void update() {
            selfMath.calculateAll();
        }

        @Override
        public void showData() {

        }

        public class SelfMath{
            private double lastMotorPos, curMotorPos, deltaPos;
            private double deltaHead;
            private ElapsedTime runTime;
            private double curTime, deltaTime, lastTime;
            public void calculateAll(){
                double outPutResolution = 288;

                curMotorPos = motorsWrapper.get("turretMotor").getMotor().getCurrentPosition();
                deltaPos = lastMotorPos - curMotorPos;
                lastMotorPos = curMotorPos;

                deltaHead = (deltaPos / outPutResolution * 2 * Math.PI);

                curTime = runTime.milliseconds();
                deltaTime = curTime - lastTime;
                lastTime = curTime;

                headVel = deltaHead / (deltaTime / 1000);

                turrelLocalHead += deltaHead;

                if(curMotorPos > outPutResolution) isInterrupted = true;
            }
        }
    }

}
