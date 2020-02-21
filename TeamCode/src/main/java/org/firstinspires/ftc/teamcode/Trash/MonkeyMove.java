package org.firstinspires.ftc.teamcode.Trash;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Exponential_Methods;

@TeleOp(name = "Monkey Move")
public class MonkeyMove extends Exponential_Methods {
    public void move(double inchesX, double inchesY, double tolerance) {
        double encoderXDistance = convertInchToEncoderOdom(inchesX) - odoWheelSideways.getCurrentPosition();
        double encoderYDistance = convertInchToEncoderOdom(inchesY) - odoWheelForwards.getCurrentPosition();
        double encoderTolerance = convertInchToEncoderOdom(tolerance);

        while ((Math.abs(encoderXDistance) > encoderTolerance || Math.abs(encoderYDistance) > encoderTolerance) && opModeIsActive()) {
            double magnitude = Math.sqrt(Math.pow(encoderXDistance, 2) + Math.pow(encoderYDistance, 2));
            double[] motorPowers = circle_to_taxicab(encoderXDistance / magnitude, encoderYDistance / magnitude, 0);

            double factor = 1;
            frontRight.setPower(factor * motorPowers[0]);
            backRight.setPower(factor * motorPowers[1]);
            backLeft.setPower(factor * motorPowers[2]);
            frontLeft.setPower(factor * motorPowers[3]);

            encoderXDistance = convertInchToEncoderOdom(inchesX) - odoWheelSideways.getCurrentPosition();
            encoderYDistance = convertInchToEncoderOdom(inchesY) - odoWheelForwards.getCurrentPosition();
        }
        setPowerDriveMotors(0);
    }
}
