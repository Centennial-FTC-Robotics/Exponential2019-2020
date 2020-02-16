package org.firstinspires.ftc.teamcode.Trash;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.util.*;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Exponential_Methods;

import java.io.File;

@TeleOp(name = "Odometry Testing", group = "TeleOp")
public class OdometryCode extends Exponential_Methods {
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        waitForStart();

        File testCases = AppUtil.getInstance().getSettingsFile("OdometryCodeTestCases.txt");
        ReadWriteFile.readFile(testCases).trim();
        double pLin = Double.parseDouble("1");
        double pRot = Double.parseDouble("1");
        double Tolerance = Double.parseDouble("1");
        telemetry.addData("pLin", pLin);
        telemetry.addData("pRot", pRot);
        telemetry.addData("Tolerance", Tolerance);
        telemetry.update();
        double x = 0;
        double y = 0;
        ElapsedTime time = new ElapsedTime();
        while (opModeIsActive()) {
            if (gamepad1.right_bumper || gamepad1.right_bumper) {
                if (gamepad1.right_stick_x > 0) {
                    x += .2;
                }
                if (gamepad1.right_stick_x < 0) {
                    x -= .2;
                }
                if (gamepad1.right_stick_y < 0) {
                    y += .2;
                }
                if (gamepad1.right_stick_y > 0) {
                    y -= .2;
                }
            } else {
                if (gamepad1.right_stick_x > 0) {
                    x += 1;
                }
                if (gamepad1.right_stick_x < 0) {
                    x -= 1;
                }
                if (gamepad1.right_stick_y < 0) {
                    y += 1;
                }
                if (gamepad1.right_stick_y > 0) {
                    y -= 1;
                }
            }
            if (gamepad1.right_stick_y != 0 || gamepad1.right_stick_x != 0) {
                wait(250);
            }
            if (gamepad1.a) {
            }
            if (gamepad1.a && time.seconds() > .5) {
                pLin = Double.parseDouble("1");
                pRot = Double.parseDouble("1");
                Tolerance = Double.parseDouble("1");
                time.reset();
            }
            telemetry.addData("pLin", pLin);
            telemetry.addData("pRot", pRot);
            telemetry.addData("Tolerance", Tolerance);
            telemetry.addData("x, y", x + ", " + y);
            telemetry.update();
        }
    }


    private double[] rototePoint(double x, double y, double angle /*in degrees*/) {
        double[] translatedPoint = new double[2];
        double angleRad = Math.PI / 180 * angle;
        translatedPoint[0] = x * Math.cos(angleRad) - y * Math.sin(angleRad);
        translatedPoint[1] = y * Math.cos(angleRad) + x * Math.sin(angleRad);
        return translatedPoint;
    }

    public void move(double xInch, double yInch, double pRot, double pLin, double iRot, double iLin, double dRot, double dLin, double maxPower, double inchesTolerence) {
        double powerMax = 0.7;
        double powerMin = -0.7;

        odoWheelForwards.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        odoWheelSideways.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        while (odoWheelSideways.isBusy() || odoWheelForwards.isBusy()) {
        }
        odoWheelForwards.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        odoWheelSideways.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // TODO: 2/11/2020 Find the odoForwards and odoSideways constants
        double odoForwardsError = 0; // the amount of encoders the forward odometry wheel rotates per radian of robot rotation,
        // negative indicates it loses encoder counts when rotating anti-clockwise
        double odoSidewaysError = 0; // the amount of encoders the sideways odometry wheel rotates per radian of robot rotation

        double initialAngle = getRotationinDimension('Z'); // In degrees
        // TODO: 2/15/2020  check if angle goes past -180 and past 180
        double xTarget = convertInchToEncoderOdom(xInch); // In encoders, relative to the field, x coordinate with the origin being the start
        double yTarget = convertInchToEncoderOdom(yInch); // In encoders, relative to the field, y coordinate with the origin being the start
        double xRobotPos = 0;
        double yRobotPos = 0;

        //IMU from -180 to 180
        double intialAngle = getRotationinDimension('Z'); // -180 to 180
        double lastAngleIMU = getRotationinDimension('Z'); // -180 to 180
        double lastAngle = getRotationinDimension('Z'); // -inf to inf
        double lastodoWheelSidewaysPosition = odoWheelSideways.getCurrentPosition();
        double lastodoWheelForwardsPosition = odoWheelForwards.getCurrentPosition();

        double tolerance = convertInchToEncoderOdom(inchesTolerence); // Tolerance in encoders
        while (opModeIsActive() && (Math.abs(xTarget - xRobotPos) > tolerance || Math.abs(yTarget - yRobotPos) > tolerance)) {

            double currentAngleIMU = getRotationinDimension('Z');
            double changeInAngle;
            if (Math.abs(lastAngleIMU - currentAngleIMU) > 300) {
                if (lastAngleIMU > currentAngleIMU) {
                    lastAngle += currentAngleIMU - lastAngleIMU + 360;
                    changeInAngle = currentAngleIMU - lastAngleIMU + 360;
                } else {
                    lastAngle += currentAngleIMU - lastAngleIMU - 360;
                    changeInAngle = currentAngleIMU - lastAngleIMU - 360;
                }
            } else {
                lastAngle += currentAngleIMU - lastAngleIMU;
                changeInAngle = currentAngleIMU - lastAngleIMU;
            }
            lastAngleIMU = currentAngleIMU;
            //telemetry.addData("angle", lastAngle);
            //telemetry.update();

            double arcDistance = Math.sqrt((odoWheelSideways.getCurrentPosition() - lastodoWheelSidewaysPosition - odoSidewaysError * (changeInAngle))
                    * (odoWheelSideways.getCurrentPosition() - lastodoWheelSidewaysPosition - odoSidewaysError * (changeInAngle))
                    + (odoWheelForwards.getCurrentPosition() - lastodoWheelForwardsPosition - odoForwardsError * (changeInAngle))
                    * (odoWheelForwards.getCurrentPosition() - lastodoWheelForwardsPosition - odoForwardsError * (changeInAngle)));
            // Since the robot rotates with the arc, the distance the odometry wheels measure is going to be the distance of the arc
            double radius = arcDistance / (Math.PI / 180 * (changeInAngle));

            // Segment of the arc is the chord that represents the total displacement of the robot as it travelled on the arc
            double angleOfSegment = (2 * lastAngle - changeInAngle) / 2;
            double distanceOfSegment = 2 * radius * Math.sin(Math.PI / 180 * (changeInAngle) / 2);


            // Rotation of the displacement to get the displacement relative to the robot
            double xDisplacement = (xTarget - xRobotPos) * Math.cos((changeInAngle) * Math.PI / 180) + (yTarget - yRobotPos) * Math.sin((changeInAngle) * Math.PI / 180); // Displacement is relative to robot
            double yDisplacement = -(xTarget - xRobotPos) * Math.sin((changeInAngle) * Math.PI / 180) + (yTarget - yRobotPos) * Math.cos((changeInAngle) / 2 * Math.PI / 180); // Displacement is relative to robot

            frontLeft.setPower(Range.clip(pLin * (yDisplacement - xDisplacement), powerMin, powerMax) - pRot * (lastAngle - initialAngle));
            frontRight.setPower(Range.clip(pLin * (yDisplacement + xDisplacement), powerMin, powerMax) + pRot * (lastAngle - initialAngle));
            backLeft.setPower(Range.clip(pLin * (yDisplacement + xDisplacement), powerMin, powerMax) - pRot * (lastAngle - initialAngle));
            backRight.setPower(Range.clip(pLin * (yDisplacement - xDisplacement), powerMin, powerMax) + pRot * (lastAngle - initialAngle));

            xRobotPos += rototePoint(distanceOfSegment * (-Math.cos(angleOfSegment * Math.PI / 180)), distanceOfSegment * (Math.sin(angleOfSegment * Math.PI / 180)), lastAngle - initialAngle)[0];
            yRobotPos += rototePoint(distanceOfSegment * (-Math.cos(angleOfSegment * Math.PI / 180)), distanceOfSegment * (Math.sin(angleOfSegment * Math.PI / 180)), lastAngle - initialAngle)[1];
            lastodoWheelSidewaysPosition = odoWheelSideways.getCurrentPosition();
            lastodoWheelForwardsPosition = odoWheelForwards.getCurrentPosition();
        }
    }
}
