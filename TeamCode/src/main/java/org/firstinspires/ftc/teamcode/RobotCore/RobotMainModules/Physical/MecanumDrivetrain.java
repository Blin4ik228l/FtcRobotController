package org.firstinspires.ftc.teamcode.RobotCore.RobotMainModules.Physical;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.RobotCore.RobotMainModules.Module;
import org.firstinspires.ftc.teamcode.RobotCore.Utils.Vector2;

public class MecanumDrivetrain implements Module {
    public final OpMode op;

    public volatile DcMotor rightB;
    public volatile DcMotor rightF;
    public volatile DcMotor leftB;
    public volatile DcMotor leftF;

    public MecanumDrivetrain(OpMode op){
        this.op = op;
    }

    @Override
    public void init() {
        rightB = op.hardwareMap.get(DcMotor.class, "rightB");
        rightF = op.hardwareMap.get(DcMotor.class, "rightF");
        leftB = op.hardwareMap.get(DcMotor.class, "leftB");
        leftF = op.hardwareMap.get(DcMotor.class, "leftF");

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
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behabior) {
        rightF.setZeroPowerBehavior(behabior);
        leftB.setZeroPowerBehavior(behabior);
        leftF.setZeroPowerBehavior(behabior);
        rightB.setZeroPowerBehavior(behabior);
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
        // TODO
        rightF.setPower(Range.clip((powerX + PowerY + heading), -1.0, 1.0));
        rightB.setPower(Range.clip((powerX - PowerY + heading), -1.0, 1.0));

        leftF.setPower(Range.clip((powerX - PowerY - heading), -1.0, 1.0));
        leftB.setPower(Range.clip((powerX + PowerY - heading), -1.0, 1.0));

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
}
