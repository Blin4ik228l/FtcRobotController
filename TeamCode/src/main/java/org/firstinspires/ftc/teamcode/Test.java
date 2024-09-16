package org.firstinspires.ftc.teamcode;

import static java.lang.Math.*;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Test {

    class Coordinates {
        double x;
        double y;

        public Coordinates(double x, double y) {
            this.x = x;
            this.y = y;
        }
    }

    Coordinates robotCoordinatesToWorldCoordinates(Coordinates robotCoordinates,
                                                   double angleRadians) {

        double resultX = robotCoordinates.x * cos(angleRadians) - robotCoordinates.y * sin(angleRadians);
        double resultY = robotCoordinates.x * sin(angleRadians) + robotCoordinates.y * сos(angleRadians);

        return new Coordinates(resultX, resultY);
    }

    Coordinates worldCoordinatesToRobotCoordinates(Coordinates worldCoordinates,
                                                   double angleRadians) {
        return robotCoordinatesToWorldCoordinates(worldCoordinates, -angleRadians);
    }

    double ticksToCm(double ticks) {
        return 0; // Тут написать что-то умное
    }

    double DISTANCE_BETWEEN_WHEELS = 20.0;

    double ticksToAngle(double ticks)  {
        double radians = ticksToCm(ticks) / (DISTANCE_BETWEEN_WHEELS / 2);
        return radians;
    }


    Coordinates globalCoordinates = new Coordinates(0,0);
    double globalAngle = 0;

    DcMotorEx leftEncoderX = null;
    DcMotorEx rightEncoderX = null;
    DcMotorEx encoderY = null;

    double leftEncoderXold = 0;
    double rightEncoderXold = 0;
    double encoderYold = 0;


    final double Y_RADIUS = 10;

    void updateOdometry( ) {
        double leftEncoderXnow = leftEncoderX.getCurrentPosition( );
        double deltaLeftEncoderX = leftEncoderXnow - leftEncoderXold;
        leftEncoderXold = leftEncoderXnow;

        double rightEncoderXnow = rightEncoderX.getCurrentPosition( );
        double deltaRightEncoderX = rightEncoderXnow - rightEncoderXold;
        rightEncoderXold = rightEncoderXnow;

        double encoderYnow = encoderY.getCurrentPosition( );
        double deltaEncoderY = encoderYnow - encoderYold;
        rightEncoderXold = encoderYnow;

        double deltaAngle = ticksToAngle(deltaRightEncoderX - deltaLeftEncoderX);

        double deltaX = ticksToCm(deltaLeftEncoderX + deltaRightEncoderX) / 2.0;
        double deltaY = ticksToCm(deltaEncoderY) - deltaAngle * Y_RADIUS;


        Coordinates deltaRobotCoordinates = new Coordinates(deltaX, deltaY);
        Coordinates deltaGlobalCoordinates = robotCoordinatesToWorldCoordinates(deltaRobotCoordinates,
                globalAngle);
        globalCoordinates.x += deltaGlobalCoordinates.x;
        globalCoordinates.y += deltaGlobalCoordinates.y;

    }

    private static final double kP = 0.0;


    DcMotorEx leftFrontMotor = null;
    DcMotorEx rightFrontMotor = null;
    DcMotorEx leftBackMotor = null;
    DcMotorEx rightBackMotor = null;


    void setPowers(double forward, double side, double angle) {
        leftFrontMotor.setPower(forward - side - angle);
        rightFrontMotor.setPower(forward + side + angle);
        leftBackMotor.setPower(forward + side - angle);
        rightBackMotor.setPower(forward -side + angle);
    }

    void goToCoordinates(Coordinates targetWorldCoordinates) {
        Coordinates error = new Coordinates(
                targetWorldCoordinates.x - globalCoordinates.x,
                targetWorldCoordinates.y - globalCoordinates.y
        );

        Coordinates targetWorldSpeed = new Coordinates(
                error.x * kP,
                error.y * kP
        );

        Coordinates targetRobotSpeed = worldCoordinatesToRobotCoordinates(
                targetWorldSpeed,
                globalAngle
        );

        setPowers(targetRobotSpeed.x, targetRobotSpeed.y, 0);
    }


    DcMotorEx motor = null;

    double targetVelocityTicks = 1000.0;

    final double kF = 1 / 2400.0;

    double targetVelX, targetVelY, targetVelAngle;


    void updateRobotVelocity( ) {
        double velocityAngle = ticksToAngle(-leftEncoderX.getVelocity() + rightEncoderX.getVelocity());
        double velocityX = ticksToCm((leftEncoderX.getVelocity() + rightEncoderX.getVelocity()) / 2.0);
        double velocityY = ticksToCm(encoderY.getVelocity()) - velocityAngle * Y_RADIUS;

        setPowers(
                (targetVelX - velocityX) * kP + targetVelX * kF,
                (targetVelY - velocityY) * kP + targetVelY * kF,
                (targetVelAngle - velocityAngle) * kP + targetVelAngle * kF
        );
    }

    ElapsedTime timer = new ElapsedTime( );
    double encoderOld = 0;

    // Ручное определение скорости
    double myGetVelocity( ) {
        double position = motor.getCurrentPosition();
        double deltaPosition = position - encoderOld;
        encoderOld = position;
        double velocity = deltaPosition / timer.seconds();
        timer.reset();
        return velocity;
    }

    double getCorrectedVelocity( ) {
        // Напрямую с контроллера
        double velocityFromController = motor.getVelocity( );

        // Посчитанная вручную примерная скорость
        double approxVelocity = myGetVelocity( );

        // Количество переполнений
        long overflowCount = round((approxVelocity - velocityFromController) / 65535.0);

        return velocityFromController + 65535.0 * overflowCount;
    }



}
