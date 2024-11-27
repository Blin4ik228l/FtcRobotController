package org.firstinspires.ftc.teamcode.RobotCore.RobotMainModules;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.RobotCore.Utils.Vector2;

public class MecanumDrivetrain implements Module {
    public final OpMode op;

    public volatile DcMotorEx rightB;
    public volatile DcMotorEx rightF;
    public volatile DcMotorEx leftB;
    public volatile DcMotorEx leftF;

    public MecanumDrivetrain(OpMode op){
        this.op = op;
    }

    @Override
    public void init() {
        rightB = op.hardwareMap.get(DcMotorEx.class, "rightB");
        rightF = op.hardwareMap.get(DcMotorEx.class, "rightF");
        leftB = op.hardwareMap.get(DcMotorEx.class, "leftB");
        leftF = op.hardwareMap.get(DcMotorEx.class, "leftF");

        rightB.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightF.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        leftF.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        leftB.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        leftF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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
    public void setVelocity(Vector2 direct, double heading){
        // TODO
        rightF.setPower(Range.clip((direct.x + direct.y + heading), -1.0, 1.0));
        rightB.setPower(Range.clip((direct.x - direct.y + heading), -1.0, 1.0));

        leftF.setPower(Range.clip((-direct.x + direct.y + heading), -1.0, 1.0));
        leftB.setPower(Range.clip((-direct.x - direct.y + heading), -1.0, 1.0));

    }

    public void setXYHeadVel(double velX, double velY, double heading){
        // TODO
        rightF.setPower(Range.clip((velX + velY + heading), -1.0, 1.0));
        rightB.setPower(Range.clip((velX - velY + heading), -1.0, 1.0));

        leftF.setPower(Range.clip((-velX + velY + heading), -1.0, 1.0));
        leftB.setPower(Range.clip((-velX - velY + heading), -1.0, 1.0));

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

    public void setVelocityTeleOp(double forward, double side, double angle){
        rightF.setPower(Range.clip((-forward - side - angle ), -1.0, 1.0));
        leftB.setPower(Range.clip((forward + side - angle ), -1.0, 1.0));
        leftF.setPower(Range.clip((forward - side - angle ), -1.0, 1.0));
        rightB.setPower(Range.clip((-forward + side - angle ), -1.0, 1.0));
    }

    public void setVelocityTeleOpHeadless(double forward, double side, double angle, double k, double u){
        rightF.setPower(Range.clip((-forward - side - angle ) * k, -1.0, 1.0));
        leftB.setPower(Range.clip((forward + side - angle ) * k, -1.0, 1.0));
        leftF.setPower(Range.clip((forward - side - angle ) * u, -1.0, 1.0));
        rightB.setPower(Range.clip((-forward + side - angle) * u, -1.0, 1.0));
    }
}
