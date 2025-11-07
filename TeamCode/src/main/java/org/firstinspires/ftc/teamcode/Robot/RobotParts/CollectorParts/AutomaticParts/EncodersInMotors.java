package org.firstinspires.ftc.teamcode.Robot.RobotParts.CollectorParts.AutomaticParts;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Modules.Module;

public class EncodersInMotors extends Module {
    public EncodersInMotors(OpMode op) {
        super(op.telemetry);
        encRight = op.hardwareMap.get(DcMotorEx.class, "flyWheelRight");
        encLeft = op.hardwareMap.get(DcMotorEx.class, "flyWheelLeft");

        encRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);                  // обновляем правый энкодер
        encLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);                  // обновляем левый энкодер

        encLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);//Запускаем
        encRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        selfMath = new SelfMath();

        telemetry.addLine("Encoders on collector Inited");
    }
    private final DcMotorEx encRight;
    private final DcMotorEx encLeft;
    private final SelfMath selfMath;
    private final double COUNTS_PER_ROUND = 28;
    public void setVelocities(double velocities){
        encLeft.setVelocity(-velocities, AngleUnit.RADIANS);
        encRight.setVelocity(velocities, AngleUnit.RADIANS);
    }
    public DcMotorEx getEncLeft() {
        return encLeft;
    }

    public DcMotorEx getEncRight() {
        return encRight;
    }
    public double curSpeed;

    public void updateAll(){
        selfMath.calculate();
    }
    public class SelfMath{
        public void calculate(){
            updateFlyWheelSpeed(true);
        }

        public void updateFlyWheelSpeed(boolean isInRadians){
            curSpeed = (encLeft.getVelocity() - encRight.getVelocity()) / (2.0 * COUNTS_PER_ROUND);

            if(isInRadians) curSpeed = (encLeft.getVelocity(AngleUnit.RADIANS) - encRight.getVelocity(AngleUnit.RADIANS)) / 2.0;

        }
    }
    public void showFLyWheelSpeed(){
        telemetry.addData("FlyWheelSpeed", encLeft.getVelocity(AngleUnit.RADIANS));
    }
}
