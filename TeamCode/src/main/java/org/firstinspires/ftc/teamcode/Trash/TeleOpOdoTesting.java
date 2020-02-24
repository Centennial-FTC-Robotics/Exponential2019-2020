package org.firstinspires.ftc.teamcode.Trash;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Exponential_Methods;
import org.firstinspires.ftc.teamcode.TeleOp.TeleOpMethods;

@TeleOp(name = "Odometry TeleOp Testing")
public class TeleOpOdoTesting extends TeleOpMethods {
    public void runOpMode()throws InterruptedException {
        super.runOpMode();
        initialAngle = getRotationInDimension('Z');
        lastodoWheelForwardsPosition = odoWheelForwards.getCurrentPosition();
        lastodoWheelSidewaysPosition = odoWheelSideways.getCurrentPosition();
        lastAngleIMU = initialAngle;
        currentAngle = initialAngle;
    }

    private double[] rototePoint(double x, double y, double angle /*in degrees*/) {
        double[] translatedPoint = new double[2];
        double angleRad = Math.toRadians(angle);
        translatedPoint[0] = x * Math.cos(angleRad) - y * Math.sin(angleRad);
        translatedPoint[1] = y * Math.cos(angleRad) + x * Math.sin(angleRad);
        return translatedPoint;
    }


    //Bearings so clockwise from position y axis
    public double getAngleBearing(double x, double y) {
        return Math.toDegrees(Math.atan2(x, y));
    }


    double xRobotPos = 0; // in terms of field
    double yRobotPos = 0; // in terms of field
    double xRobotVel = 0;
    double yRobotVel = 0;
    double areaXDis = 0; // perspective of field
    double areaYDis = 0; // perspective of field


    double initialAngle; // -180 to 180
    double lastAngleIMU; // -180 to 180
    double currentAngle; // -inf to inf

    ElapsedTime time = new ElapsedTime();
    double lastodoWheelSidewaysPosition;
    double lastodoWheelForwardsPosition;

    public void driveTrain() {
        super.driveTrain();
        odoWheelForwards.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        odoWheelSideways.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        odoWheelForwards.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        odoWheelSideways.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // TODO: 2/11/2020 Find the odoForwards and odoSideways constants
        double odoForwardsError = 0; // the amount of encoders the forward odometry wheel rotates per radian of robot rotation,
        // negative indicates it loses encoder counts when rotating anti-clockwise
        double odoSidewaysError = 0; // the amount of encoders the sideways odometry wheel rotates per radian of robot rotation


        //IMU from -180 to 180

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

        double arcDistance = Math.sqrt((odoWheelSideways.getCurrentPosition() - lastodoWheelSidewaysPosition - odoSidewaysError * (changeInAngle))
                * (odoWheelSideways.getCurrentPosition() - lastodoWheelSidewaysPosition - odoSidewaysError * (changeInAngle))
                + (odoWheelForwards.getCurrentPosition() - lastodoWheelForwardsPosition - odoForwardsError * (changeInAngle))
                * (odoWheelForwards.getCurrentPosition() - lastodoWheelForwardsPosition - odoForwardsError * (changeInAngle)));

        if (Math.abs(changeInAngle) <= 0.001) {
            // Since the robot rotates with the arc, the distance the odometry wheels measure is going to be the distance of the arc
            double radius = Math.abs(arcDistance / (Math.PI / 180 * changeInAngle));
            // Segment of the arc is the chord that represents the total displacement of the robot as it travelled on the arc

            double angleOffBearing = getAngleBearing(gamepad1.left_stick_x, gamepad1.left_stick_y);
            xRobotPos += rototePoint(radius * (1 - Math.cos(changeInAngle * Math.PI / 180)), radius * (Math.sin(changeInAngle * Math.PI / 180)), currentAngle - initialAngle - angleOffBearing - changeInAngle)[0];
            yRobotPos += rototePoint(radius * (1 - Math.cos(changeInAngle * Math.PI / 180)), radius * (Math.sin(changeInAngle * Math.PI / 180)), currentAngle - initialAngle - angleOffBearing - changeInAngle)[1];
            xRobotVel = rototePoint(radius * (1 - Math.cos(changeInAngle * Math.PI / 180)), radius * (Math.sin(changeInAngle * Math.PI / 180)), currentAngle - initialAngle - angleOffBearing - changeInAngle)[0] / time.seconds();
            yRobotVel = rototePoint(radius * (1 - Math.cos(changeInAngle * Math.PI / 180)), radius * (Math.sin(changeInAngle * Math.PI / 180)), currentAngle - initialAngle - angleOffBearing - changeInAngle)[1] / time.seconds();
        } else {
            xRobotPos += rototePoint(odoWheelSideways.getCurrentPosition() - lastodoWheelSidewaysPosition, odoWheelForwards.getCurrentPosition() - lastodoWheelForwardsPosition, currentAngle - initialAngle)[0];
            yRobotPos += rototePoint(odoWheelSideways.getCurrentPosition() - lastodoWheelSidewaysPosition, odoWheelForwards.getCurrentPosition() - lastodoWheelForwardsPosition, currentAngle - initialAngle)[1];
            xRobotVel = rototePoint(odoWheelSideways.getCurrentPosition() - lastodoWheelSidewaysPosition, odoWheelForwards.getCurrentPosition() - lastodoWheelForwardsPosition, currentAngle - initialAngle)[0] / time.seconds();
            yRobotVel = rototePoint(odoWheelSideways.getCurrentPosition() - lastodoWheelSidewaysPosition, odoWheelForwards.getCurrentPosition() - lastodoWheelForwardsPosition, currentAngle - initialAngle)[1] / time.seconds();
        }
        // Segment of the arc is the chord that represents the total displacement of the robot as it travelled on the arc

        // Rotation of the displacement to get the displacement relative to the robot

        // TODO: 2/19/2020 figure out if dLin is supposed to be negative or positive
            /*
            double magnitude = Math.sqrt(xDisplacement*xDisplacement+yDisplacement*yDisplacement);
            double[] answer = circle_to_taxicab(xDisplacement/magnitude, xDisplacement/magnitude, 0);
            double speed = 1;
            frontRight.setPower(speed*answer[0]);
            backRight.setPower(speed*answer[1]);
            backLeft.setPower(speed*answer[2]);
            frontLeft.setPower(speed*answer[3]);
            */
        lastodoWheelSidewaysPosition = odoWheelSideways.getCurrentPosition();
        lastodoWheelForwardsPosition = odoWheelForwards.getCurrentPosition();
        telemetry.addData("Change in Angle", changeInAngle);
        telemetry.addData("Arc Length", arcDistance);
        telemetry.addData("Orientation", currentAngle - initialAngle);
        telemetry.addData("x Robot Position", convertEncoderToInchOdom(xRobotPos));
        telemetry.addData("y Robot Position", convertEncoderToInchOdom(yRobotPos));
        telemetry.addData("Timer", time.seconds());
        time.reset();
        telemetry.update();
    }
}