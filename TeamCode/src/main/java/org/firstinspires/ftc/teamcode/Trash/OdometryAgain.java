package org.firstinspires.ftc.teamcode.Trash;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Exponential_Methods;
import org.firstinspires.ftc.teamcode.TeleOp.TeleOpMethods;

import java.io.File;

@TeleOp(name = "Odometry Version 2", group = "TeleOp")
public class OdometryAgain extends Exponential_Methods {
    class Decay {
        final static double Odometry_Sideways_Error = -15.8524; // encoders per degree
        final static double Odometry_Forwards_Error = -167.92024; // encoders per degree
        // TODO: 2/25/2020 change to see if they are supposed to be positive or negative
        // class to store info about robot for the move method
        double lastOdoSide;
        double lastOdoForward;
        double changeOdoSide; // change in the odometry encoders in between loop
        double changeOdoForward; // change in the odometry encoders in between loop

        double initialAngle; // -180 to 180
        double lastAngleIMU; // -180 to 180
        double currentAngle; // -inf to inf
        double changeInAngle; // change in angle between two consecutive loops

        ElapsedTime period = new ElapsedTime();

        Decay() {
            odoWheelForwards.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            odoWheelSideways.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            odoWheelForwards.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            odoWheelSideways.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            lastOdoSide = 0;
            lastOdoForward = 0;
            changeOdoSide = 0;
            changeOdoForward = 0;

            initialAngle = getRotationInDimension('Z');
            lastAngleIMU = initialAngle;
            currentAngle = initialAngle;
            changeInAngle = 0;

        }

        public void refresh() {
            // refreshes the information on the robot
            // Updates the angle so that currentAngle goes from negative infinity to positive
            // infinity rather than have if clip off at -180 to 180
            double currentAngleIMU = getRotationInDimension('Z');
            if (Math.abs(lastAngleIMU - currentAngleIMU) > 300) {
                // if the angle crossed the negative x axis
                if (lastAngleIMU > currentAngleIMU) {
                    changeInAngle = currentAngleIMU - lastAngleIMU + 360;
                } else {
                    changeInAngle = currentAngleIMU - lastAngleIMU - 360;
                }
            } else {
                // normal
                changeInAngle = currentAngleIMU - lastAngleIMU;
            }
            currentAngle += changeInAngle;
            lastAngleIMU = currentAngleIMU;

            // sleep(100);
            // updates the odometry
            changeOdoSide = odoWheelSideways.getCurrentPosition() - lastOdoSide;
            changeOdoForward = odoWheelForwards.getCurrentPosition() - lastOdoForward;
            lastOdoSide = odoWheelSideways.getCurrentPosition();
            lastOdoForward = odoWheelForwards.getCurrentPosition();

            period.reset();
        }

        public double[] getDisplacement() {
            // displacement relative to the robot
            // first element is x, second is y

            // finds the actual displacement
            //if (Math.abs(changeInAngle) == 0.0) {
            return new double[]{changeOdoSide - changeInAngle * Odometry_Sideways_Error, changeOdoForward - changeInAngle * Odometry_Forwards_Error};
            /*} else {
                // length of the arc that the robot has travelled
                double arcDistance = Math.sqrt(Math.pow(changeOdoSide - changeInAngle * Odometry_Sideways_Error, 2)
                        + Math.pow(changeOdoForward - changeInAngle * Odometry_Forwards_Error, 2));

                // radius of circle that contains the arc
                double radius = arcDistance / (Math.toRadians(Math.abs(changeInAngle)));

                // angle of the direction of the robot velocity using the motor power
                double angle = -Math.atan2(changeOdoSide, changeOdoForward);
                telemetry.addData("Angle", angle);

                // sleep(500);
                //Keertic's math that gave the arc in terms of x and y
                return rotatePoint(radius * Math.cos(Math.toRadians(changeInAngle)) - radius, radius * Math.sin(Math.toRadians(changeInAngle)), Math.toDegrees(angle));
            }*/
        }
    }

    private static double[] rotatePoint(double x, double y, double angle /*in degrees*/) {
        double[] translatedPoint = new double[2];
        double angleRad = Math.PI / 180 * angle;
        translatedPoint[0] = x * Math.cos(angleRad) - y * Math.sin(angleRad);
        translatedPoint[1] = y * Math.cos(angleRad) + x * Math.sin(angleRad);
        return translatedPoint;
    }

    public double distance(double x, double y, double xTarget, double yTarget) {
        return Math.sqrt(Math.pow(x - xTarget, 2) + Math.pow(y - yTarget, 2));
    }

    public void move(double X, double Y, double P, double I, double D, double tolerance) {
        double xTarget = convertInchToEncoderOdom(X);
        double yTarget = convertInchToEncoderOdom(Y);
        tolerance = convertInchToEncoderOdom(tolerance);


        // x and y are zero at the start of the move method, robot points in positive y
        double xField = 0;
        double yField = 0;
        Decay decay = new Decay();

        String input = "";


        while (opModeIsActive() && distance(xField, yField, xTarget, yTarget) > tolerance) {
            decay.refresh();

            // starts with displacement in terms of the robot
            double[] displacement = decay.getDisplacement();
            // changes displacement to get it in terms of the field
            displacement = rotatePoint(displacement[0], displacement[1], decay.currentAngle - decay.initialAngle - decay.changeInAngle);
            xField += displacement[0];
            yField += displacement[1];

            // telemetry
            telemetry.addData("Coordinate Field", convertEncoderToInchOdom(xField) + ", " + convertEncoderToInchOdom(yField));
            telemetry.addData("Displacement", convertInchToEncoderOdom((displacement[0])) + ", " + convertInchToEncoderOdom(displacement[1]));
            telemetry.addData("time", decay.period.seconds());
            //telemetry.addData("error forwards", odoWheelForwards.getCurrentPosition()/decay.currentAngle);
            //telemetry.addData("error sideways", odoWheelSideways.getCurrentPosition()/decay.currentAngle);
            telemetry.update();
            input += " (" + displacement[0] + ", " + displacement[1] + ") \n";

            // sets the robot to respond to the gamepad
            // TODO take out the gamepad actions
            double[] answer = circle_to_taxicab(gamepad1.left_stick_x, gamepad1.left_stick_y, 0.8 * gamepad1.right_stick_x);
            double factor = 1;
            if (gamepad1.left_bumper) {
                // if left bumper is pressed, reduce the motor speed
                factor = 0.5;
            }
            if (gamepad1.right_bumper) {
                // if right bumper is pressed, reduce the motor speed
                factor = 0.25;
            }
            frontRight.setPower(factor * answer[0]);
            backRight.setPower(factor * answer[1]);
            backLeft.setPower(factor * answer[2]);
            frontLeft.setPower(factor * answer[3]);
        }
        File test = AppUtil.getInstance().getSettingsFile("test.txt");
        ReadWriteFile.writeFile(test, input);


    }

    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        move(10000, 10000, 0, 0, 0, 0);
    }
}
