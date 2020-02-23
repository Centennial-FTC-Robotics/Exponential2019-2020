package org.firstinspires.ftc.teamcode.Trash;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.util.*;

import org.firstinspires.ftc.teamcode.Exponential_Methods;

@TeleOp(name = "Odometry Testing", group = "TeleOp")
public class OdometryCode extends Exponential_Methods {
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        waitForStart();
        double pLin = Double.parseDouble("1");
        double pRot = Double.parseDouble("1");
        double Tolerance = Double.parseDouble("1");
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
                move(x, y, 0.02, 1.0 / 1200, 0, 0.0015, 0, 0, 0.5, 2);
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


    private double[] rotatePoint(double x, double y, double angle /*in degrees*/) {
        double[] translatedPoint = new double[2];
        double angleRad = Math.PI / 180 * angle;
        translatedPoint[0] = x * Math.cos(angleRad) - y * Math.sin(angleRad);
        translatedPoint[1] = y * Math.cos(angleRad) + x * Math.sin(angleRad);
        return translatedPoint;
    }


    //Bearings so clockwise from position y axis
    public double getAngleBearing(double x, double y){
        if (y==0){
            if(x>0){
                return 90;
            } else {
                return -90;
            }
        }
        if(y>0){
            return Math.atan(x/y);
        } else {
            return Math.atan(x/y)+180;
        }
    }


    public void move(double xInch, double yInch, double pRot, double pLin, double iRot, double iLin, double dRot, double dLin, double maxPower, double inchesTolerence) {

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

        double xTarget = convertInchToEncoderOdom(xInch); // In encoders, relative to the field, x coordinate with the origin being the start
        double yTarget = convertInchToEncoderOdom(yInch); // In encoders, relative to the field, y coordinate with the origin being the start
        double xRobotPos = 0; // in terms of field
        double yRobotPos = 0; // in terms of field
        double xRobotVel = 0;
        double yRobotVel = 0;

        //IMU from -180 to 180
        double initialAngle = getRotationInDimension('Z'); // -180 to 180
        double lastAngleIMU = initialAngle; // -180 to 180
        double currentAngle = initialAngle; // -inf to inf
        double lastodoWheelSidewaysPosition = odoWheelSideways.getCurrentPosition();
        double lastodoWheelForwardsPosition = odoWheelForwards.getCurrentPosition();

        double tolerance = convertInchToEncoderOdom(inchesTolerence); // Tolerance in encoders
        double areaXDis = 0; // perspective of field
        double areaYDis = 0; // perspective of field
        ElapsedTime time = new ElapsedTime();
        while (opModeIsActive() && (Math.abs(xTarget - xRobotPos) > tolerance || Math.abs(yTarget - yRobotPos) > tolerance)) {
            double currentAngleIMU = getRotationInDimension('Z');
            double changeInAngle;

            // Allows angle to go greater than 180 and less than -180
            if (Math.abs(lastAngleIMU - currentAngleIMU) > 300) {
                if (lastAngleIMU > currentAngleIMU) {
                    currentAngle += currentAngleIMU - lastAngleIMU + 360;
                    changeInAngle = currentAngleIMU - lastAngleIMU + 360;
                } else {
                    currentAngle += currentAngleIMU - lastAngleIMU - 360;
                    changeInAngle = currentAngleIMU - lastAngleIMU - 360;
                }
            } else {
                currentAngle += currentAngleIMU - lastAngleIMU;
                changeInAngle = currentAngleIMU - lastAngleIMU;
            }
            lastAngleIMU = currentAngleIMU;
            //telemetry.addData("angle", lastAngle);
            //telemetry.update();
            double sidewaysOdoDisplacement = odoWheelSideways.getCurrentPosition() - lastodoWheelSidewaysPosition;
            double forwardsOdoDisplacement = odoWheelForwards.getCurrentPosition() - lastodoWheelForwardsPosition;
            double arcDistance = Math.sqrt((sidewaysOdoDisplacement - odoSidewaysError * (changeInAngle))
                    * (sidewaysOdoDisplacement - odoSidewaysError * (changeInAngle))
                    + (forwardsOdoDisplacement - odoForwardsError * (changeInAngle))
                    * (forwardsOdoDisplacement - odoForwardsError * (changeInAngle)));

            if (changeInAngle != 0) {
                // Since the robot rotates with the arc, the distance the odometry wheels measure is going to be the distance of the arc
                double radius = Math.abs(arcDistance / (Math.PI / 180 * changeInAngle));
                // Segment of the arc is the chord that represents the total displacement of the robot as it travelled on the arc
                double xChord = radius * (1 - Math.cos(changeInAngle * Math.PI / 180));
                double yChord = radius * (Math.sin(changeInAngle * Math.PI / 180));
                double angleOffBearing = getAngleBearing(xTarget-xRobotPos, yTarget-yRobotPos);

                xRobotPos += rotatePoint(xChord, yChord, currentAngle - changeInAngle - initialAngle - angleOffBearing)[0];
                yRobotPos += rotatePoint(xChord, yChord, currentAngle - changeInAngle - initialAngle - angleOffBearing)[1];
                xRobotVel = rotatePoint(xChord, yChord, currentAngle - changeInAngle - initialAngle - angleOffBearing)[0] / time.seconds();
                yRobotVel = rotatePoint(xChord, yChord, currentAngle - changeInAngle - initialAngle - angleOffBearing)[1] / time.seconds();

            } else {
                xRobotPos += rotatePoint(sidewaysOdoDisplacement, forwardsOdoDisplacement, currentAngle - initialAngle)[0];
                yRobotPos += rotatePoint(sidewaysOdoDisplacement, forwardsOdoDisplacement, currentAngle - initialAngle)[1];
                xRobotVel = rotatePoint(sidewaysOdoDisplacement, forwardsOdoDisplacement, currentAngle - initialAngle)[0] / time.seconds();
                yRobotVel = rotatePoint(sidewaysOdoDisplacement, forwardsOdoDisplacement, currentAngle - initialAngle)[1] / time.seconds();
            }
            // Segment of the arc is the chord that represents the total displacement of the robot as it travelled on the arc

            // Rotation of the displacement to get the displacement relative to the robot
            double xDisplacement = (xTarget - xRobotPos) * Math.cos(-(currentAngle - initialAngle) * Math.PI / 180) - (yTarget - yRobotPos) * Math.sin(-(currentAngle + initialAngle) * Math.PI / 180); // Displacement is relative to robot
            double yDisplacement = (xTarget - xRobotPos) * Math.sin(-(currentAngle - initialAngle) * Math.PI / 180) + (yTarget - yRobotPos) * Math.cos(-(currentAngle + initialAngle) * Math.PI / 180); // Displacement is relative to robot

            areaXDis += (xTarget - xRobotPos) * time.seconds(); // in terms of the field
            areaYDis += (yTarget - yRobotPos) * time.seconds(); // in terms of the field


            // TODO: 2/19/2020 figure out if dLin is supposed to be negative or positive
            frontLeft.setPower(
                    Range.clip(pLin * (yDisplacement - xDisplacement)
                            + iLin * (rotatePoint(areaXDis, areaYDis, -currentAngle + initialAngle)[1] - rotatePoint(areaXDis, areaYDis, -currentAngle + initialAngle)[0])
                            + dLin * (rotatePoint(xRobotVel, yRobotVel, -currentAngle + initialAngle)[1] - rotatePoint(xRobotVel, yRobotVel, -currentAngle + initialAngle)[0])
                            - pRot * (-currentAngle + initialAngle), -maxPower, maxPower));
            frontRight.setPower(
                    Range.clip(pLin * (yDisplacement + xDisplacement)
                            + iLin * (rotatePoint(areaXDis, areaYDis, -currentAngle + initialAngle)[1] + rotatePoint(areaXDis, areaYDis, -currentAngle + initialAngle)[0])
                            + dLin * (rotatePoint(xRobotVel, yRobotVel, -currentAngle + initialAngle)[1] + rotatePoint(xRobotVel, yRobotVel, -currentAngle + initialAngle)[0])
                            + pRot * (-currentAngle + initialAngle), -maxPower, maxPower));
            backLeft.setPower(
                    Range.clip(pLin * (yDisplacement + xDisplacement)
                            + iLin * (rotatePoint(areaXDis, areaYDis, -currentAngle + initialAngle)[1] + rotatePoint(areaXDis, areaYDis, -currentAngle + initialAngle)[0])
                            + dLin * (rotatePoint(xRobotVel, yRobotVel, -currentAngle + initialAngle)[1] + rotatePoint(xRobotVel, yRobotVel, -currentAngle + initialAngle)[0])
                            - pRot * (-currentAngle + initialAngle), -maxPower, maxPower));
            backRight.setPower(
                    Range.clip(pLin * (yDisplacement - xDisplacement)
                            + iLin * (rotatePoint(areaXDis, areaYDis, -currentAngle + initialAngle)[1] - rotatePoint(areaXDis, areaYDis, -currentAngle + initialAngle)[0])
                            + dLin * (rotatePoint(xRobotVel, yRobotVel, -currentAngle + initialAngle)[1] - rotatePoint(xRobotVel, yRobotVel, -currentAngle + initialAngle)[0])
                            + pRot * (-currentAngle + initialAngle), -maxPower, maxPower));
            /*
            double magnitude = Math.sqrt(xDisplacement*xDisplacement+yDisplacement*yDisplacement);
            double[] answer = circle_to_taxicab(xDisplacement/magnitude, xDisplacement/magnitude, 0);
            double speed = 1;
            frontRight.setPower(speed * answer[0]);
            backRight.setPower(speed * answer[1]);
            backLeft.setPower(speed * answer[2]);
            frontLeft.setPower(speed * answer[3]);
            */

            lastodoWheelSidewaysPosition = odoWheelSideways.getCurrentPosition();
            lastodoWheelForwardsPosition = odoWheelForwards.getCurrentPosition();
            telemetry.addData("Orientation", currentAngle - initialAngle);
            telemetry.addData("Robot Displacement", (xTarget-xRobotPos)+", "+(yTarget-yRobotPos));
            telemetry.addData("Robot Velocity", (xRobotVel)+", "+(yRobotVel));
            telemetry.addData("Robot Area", (areaXDis)+", "+(areaYDis));
            telemetry.update();

            time.reset();
        }
        setPowerDriveMotors(0);
    }
}