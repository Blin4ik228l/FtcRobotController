package org.firstinspires.ftc.teamcode.Robot.RobotParts.DrivetrainParts;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Modules.Module;

public class MotorsOnDrivetrain extends Module {

    public MotorsOnDrivetrain(OpMode op) {
        super(op.telemetry);
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

        rightF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        telemetry.addLine("Motors on drivetrain Inited");
    }
    private final DcMotor leftB;
    private final DcMotor leftF;
    private final DcMotor rightB;
    private final DcMotor rightF;

    public DcMotor getLeftB() {
        return leftB;
    }

    public DcMotor getLeftF() {
        return leftF;
    }

    public DcMotor getRightB() {
        return rightB;
    }

    public DcMotor getRightF() {
        return rightF;
    }
}
