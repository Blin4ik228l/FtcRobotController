package org.firstinspires.ftc.teamcode.Robot.RobotParts.CollectorParts;

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

        encRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);// обновляем правый энкодер
        encLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);// обновляем левый энкодер

        encLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);//Запускаем
        encRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addLine("Encoders on collector Inited");
    }
    private final DcMotorEx encRight;
    private final DcMotorEx encLeft;
    private final double COUNTS_PER_ROUND = 28;
    public double velocities;


    public void setVelocities(double speed) {
        encLeft.setVelocity(speed, AngleUnit.RADIANS);
        encRight.setVelocity(-speed, AngleUnit.RADIANS);
    }

    public double getVelocity(){
        double speed = Math.signum(encLeft.getVelocity()) * (Math.abs(encLeft.getVelocity(AngleUnit.RADIANS) + Math.abs(encRight.getVelocity(AngleUnit.RADIANS)))) / 2.0;

        return encLeft.getVelocity() != 0 ?
                encRight.getVelocity() != 0  ? speed : encLeft.getVelocity(AngleUnit.RADIANS) :
                encRight.getVelocity() != 0 ? encRight.getVelocity(AngleUnit.RADIANS) : 0;
    }
    public DcMotorEx getEncLeft() {
        return encLeft;
    }
    public DcMotorEx getEncRight() {
        return encRight;
    }

    public void showData(){
        telemetry.addLine("Encoders in collector motors")
                .addData("FlyWheelSpeed overall","%.2f", getVelocity())
                .addData("Left motor speed","%.2f", encLeft.getVelocity(AngleUnit.RADIANS))
                .addData("Right motor speed","%.2f", encRight.getVelocity(AngleUnit.RADIANS));
        telemetry.addLine();
    }
}
