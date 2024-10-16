package org.firstinspires.ftc.teamcode.RobotCore.RobotSubsystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Utils.Vector2;

public class MecanumDrivetrain implements Subsystem{
    public final OpMode op;

    public DcMotorEx rightB;
    public DcMotorEx rightF;
    public DcMotorEx leftB;
    public DcMotorEx leftF;

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
        rightF.setPower(Range.clip((-direct.x - direct.y - heading), -1.0, 1.0));
        leftB.setPower(Range.clip((direct.x + direct.y - heading), -1.0, 1.0));
        leftF.setPower(Range.clip((direct.x - direct.y - heading), -1.0, 1.0));
        rightB.setPower(Range.clip((-direct.x + direct.y - heading), -1.0, 1.0));
    }

    public void brakeMotors(){
        rightF.setPower(0);
        leftB.setPower(0);
        leftF.setPower(0);
        rightB.setPower(0);
    }
}
