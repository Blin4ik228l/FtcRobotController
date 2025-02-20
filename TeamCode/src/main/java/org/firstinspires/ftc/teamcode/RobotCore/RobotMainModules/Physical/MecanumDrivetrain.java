package org.firstinspires.ftc.teamcode.RobotCore.RobotMainModules.Physical;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.RobotCore.RobotMainModules.Module;
import org.firstinspires.ftc.teamcode.RobotCore.Utils.Vector2;

import java.util.Base64;

public class MecanumDrivetrain implements Module {
    public final OpMode op;

    public volatile DcMotor rightB;//encM
    public volatile DcMotor rightF;
    public volatile DcMotor leftB;//encR
    public volatile DcMotor leftF;//ecnL
    public volatile DcMotor led;
    public volatile DcMotorEx right;

    public MecanumDrivetrain(OpMode op){
        this.op = op;
    }

    @Override
    public void init() {
        rightB = op.hardwareMap.get(DcMotor.class, "rightB");
        rightF = op.hardwareMap.get(DcMotor.class, "rightF");
        leftB = op.hardwareMap.get(DcMotor.class, "leftB");
        leftF = op.hardwareMap.get(DcMotor.class, "leftF");

        led = op.hardwareMap.get(DcMotor.class, "led");

        rightB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightB.setDirection(DcMotorSimple.Direction.FORWARD);
        rightF.setDirection(DcMotorSimple.Direction.FORWARD);
        leftB.setDirection(DcMotorSimple.Direction.REVERSE);
        leftF.setDirection(DcMotorSimple.Direction.REVERSE);

//        leftF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        leftF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        offMotors();
        brakeMotors();
        offLed();

        op.telemetry.addLine("Drivetrain Inited");
    }

    public void offLed(){
        led.setPower(0);
    }

    public void onLed(){
        led.setPower(-0.7);
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        rightF.setZeroPowerBehavior(behavior);
        leftB.setZeroPowerBehavior(behavior);
        leftF.setZeroPowerBehavior(behavior);
        rightB.setZeroPowerBehavior(behavior);
    }

    // Распределение требуемой скорости и направления движения робота на скорость колес
    public void setPower(Vector2 power, double heading){
        // TODO
        rightF.setPower(Range.clip((power.x + power.y - heading), -1.0, 1.0));
        rightB.setPower(Range.clip((power.x - power.y - heading), -1.0, 1.0));

        leftF.setPower(Range.clip((power.x - power.y + heading), -1.0, 1.0));
        leftB.setPower(Range.clip((power.x + power.y + heading), -1.0, 1.0));

    }

    public synchronized void setXYHeadVel(double powerX, double PowerY, double heading){
        double maxV = 0.65;

        double minVAngle = 0.15;

        if(Math.abs(heading) < minVAngle) heading = minVAngle * Math.signum(heading);

        // TODO
        rightF.setPower(Range.clip((powerX + PowerY + heading), -maxV, maxV));
        rightB.setPower(Range.clip((powerX - PowerY + heading), -maxV, maxV));

        leftF.setPower(Range.clip((powerX - PowerY - heading), -maxV, maxV));
        leftB.setPower(Range.clip((powerX + PowerY - heading), -maxV, maxV));

    }

    public void brakeMotors(){
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void floatMotors(){
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void offMotors(){
        rightF.setPower(0);
        leftB.setPower(0);
        leftF.setPower(0);
        rightB.setPower(0);
    }

    public synchronized void setPowerTeleOp(double forward, double side, double angle){
        rightF.setPower(Range.clip((forward - side - angle ), -1.0, 1.0));
        rightB.setPower(Range.clip((forward + side - angle ), -1.0, 1.0));

        leftB.setPower(Range.clip((forward - side + angle ), -1.0, 1.0));
        leftF.setPower(Range.clip((forward + side + angle ), -1.0, 1.0));
    }

    public synchronized void setPowerTeleOpHeadless(double forward, double side, double angle, double k, double u){
        rightF.setPower(Range.clip((forward - side - angle ) * k, -1.0, 1.0));
        leftB.setPower(Range.clip((forward - side + angle ) * k, -1.0, 1.0));
        leftF.setPower(Range.clip((forward + side + angle ) * u, -1.0, 1.0));
        rightB.setPower(Range.clip((forward + side - angle) * u, -1.0, 1.0));
    }

    public synchronized void getMotorsPower(){
        op.telemetry.addLine("Motors power")
                .addData("\nrightF", rightF.getPower())
                .addData("\nrightB", rightB.getPower())
                .addData("\nleftF", leftF.getPower())
                .addData("\nleftB", leftB.getPower());
        op.telemetry.addLine();
    }
}
