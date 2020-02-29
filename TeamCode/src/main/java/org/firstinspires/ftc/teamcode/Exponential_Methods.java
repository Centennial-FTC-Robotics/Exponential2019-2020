package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.internal.tfod.Timer;
import org.firstinspires.ftc.teamcode.TeleOp.TeleOpMethods;

import java.util.List;

import static org.firstinspires.ftc.robotcore.external.tfod.TfodRoverRuckus.TFOD_MODEL_ASSET;
import static org.firstinspires.ftc.teamcode.TeleOp.TeleOpMethods.ROTATE_TO_MOVE_RATIO;

public abstract class Exponential_Methods extends Exponential_Hardware_Initializations {

    //tensor flow and vuforia stuff
    private static final String VUFORIA_KEY = "AQmuIUP/////AAAAGR6dNDzwEU07h7tcmZJ6YVoz5iaF8njoWsXQT5HnCiI/oFwiFmt4HHTLtLcEhHCU5ynokJgYSvbI32dfC2rOvqmw81MMzknAwxKxMitf8moiK62jdqxNGADODm/SUvu5a5XrAnzc7seCtD2/d5bAIv1ZuseHcK+oInFHZTi+3BvhbUyYNvnVb0tQEAv8oimzjiQW18dSUcEcB/d6QNGDvaDHpxuRCJXt8U3ShJfBWWQEex0Vp6rrb011z8KxU+dRMvGjaIy+P2p5GbWXGJn/yJS9oxuwDn3zU6kcQoAwI7mUgAw5zBGxxM+P35DoDqiOja6ST6HzDszHxClBm2dvTRP7C4DEj0gPkhX3LtBgdolt";
    public VuforiaLocalizer vuforia; //Vuforia localization engine
    public TFObjectDetector tfod; //Tensor Flow Object Detection engine
    private int cameraMonitorViewId;

    public SkystoneDetector skystoneDetector;

    public static final String TFOD_MODEL_ASSET = "detect.tflite";
    public static final String LABEL_FIRST_ELEMENT = "Stone";
    public static final String LABEL_SECOND_ELEMENT = "Skystone";
    public static final int MIDDLE_SCREEN = 640;
    public static final int TOP_MIDDLE_SCREEN = 360;

    public static final double TILE_LENGTH = 24;
    public static final double ROBOT_LENGTH = 18;
    public static final double BLOCK_LENGTH = 8;
    public static final double FOUNDATION_LENGTH = 34.5;
    public static final double FOUNDATION_WIDTH = 18.5;
    public static final double FOUNDATION_AWAY_FROM_WALL = 4;

    public static final double FOUNDATION_POSITION = TILE_LENGTH * 6 - ROBOT_LENGTH - 4 - FOUNDATION_LENGTH / 2;
    public static final double MIDDLE_OF_TILE = (TILE_LENGTH - ROBOT_LENGTH) / 2;

    public static final double FOUNDATION_POSITION_MOVETO = FOUNDATION_POSITION - 3 * TILE_LENGTH;

    public static final int slideMax = 2250;
    public static final int slideMin = -425;

    public static final double MAX_POWER = .6;

    public static double initialAngleOffset; //how far away the front of robot is from 90 deg

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        initializeIMU();

        //initVuforia();
        //initTfod();

    }

    public static void margaret() {
        System.out.println("fuck you");
    }

    //-------------- INITIALIZATION -------------- (organization)
    public void initializeIMU() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);
        while (opModeIsActive() && !imu.isGyroCalibrated()) ;
        resetOrientation();
    }

    public void initVuforia() {
        //create parameter object and pass it to create Vuforia engine
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    public void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.8;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

    //-------------- BASIC -------------- (organization)

    public int convertInchToEncoder(double inches) {
        double conversion = 43.4653423;
        int encoderValue = (int) Math.round(conversion * inches);
        return encoderValue;
    }

    public int convertInchToEncoderSlides(double inches) {
        double circumference = 5.30929;
        double encoderToIn = 537.6;
        double conversion = encoderToIn / circumference;
        int encoderValue = (int) Math.round(conversion * inches);
        return encoderValue;
    }

    public int convertInchToEncoderOdom(double inches) {
        return (int) (8192 / (2 * Math.PI) * inches);
    }

    public double convertEncoderToInchOdom(double encoder) {
        return (encoder) / (8192 / (2 * Math.PI));
    }

    public double getAngleDist(double targetAngle, double currentAngle) {
        double angleDifference = currentAngle - targetAngle;
        if (Math.abs(angleDifference) > 180) {
            angleDifference = 360 - Math.abs(angleDifference);
        } else {
            angleDifference = Math.abs(angleDifference);
        }

        return angleDifference;
    }

    public int getAngleDir(double targetAngle, double currentAngle) {

        double angleDifference = currentAngle - targetAngle;
        int angleDir = (int) (angleDifference / Math.abs(angleDifference));

        if (Math.abs(angleDifference) > 180) {
            angleDir *= -1;
        }

        return angleDir;
    }

    public void resetMotorEncoder(DcMotor motor) {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    //-------------- STATUS -------------- (organization)

    public boolean motorsBusy() {
        return driveMotors[0].isBusy() || driveMotors[1].isBusy() || driveMotors[2].isBusy() || driveMotors[3].isBusy();
    }

    public void waitForMotors() {
        while (motorsBusy() && opModeIsActive()) {
        }
    }

    public void resetDriveMotorEncoders() {
        for (DcMotor motor : driveMotors)
            resetMotorEncoder(motor);
    }

    public void resetOrientation() {
        updateOrientation();
        initialHeading = orientation.firstAngle;
        initialRoll = orientation.secondAngle;
        initialPitch = orientation.thirdAngle;
    }


    public void updateOrientation() {
        orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    }

    public double convertNormDegree(double degree) {
        if (degree < 0)
            return degree + 360;
        else
            return degree;
    }

    public double getAngle() {
        return convertNormDegree(getRotationInDimension('Z'));
    }

    public double getRotationInDimension(char dimension) {
        updateOrientation();
        switch (Character.toUpperCase(dimension)) {
            case 'X':
                return AngleUnit.normalizeDegrees(orientation.secondAngle - initialPitch);
            case 'Y':
                return AngleUnit.normalizeDegrees(orientation.thirdAngle - initialRoll);
            case 'Z':
                return AngleUnit.normalizeDegrees(orientation.firstAngle - initialHeading);
        }
        return 0;
    }

    public boolean hasBlock() {
        return sensorDistance.getDistance(DistanceUnit.INCH) < 1;
    }


    //-------------- SET DRIVE MOTOR POWER -------------- (organization)

    public void setPowerDriveMotors(double frontRight, double backRight, double backLeft, double frontLeft) {
        super.frontRight.setPower(frontRight);
        super.frontLeft.setPower(frontLeft);
        super.backLeft.setPower(backLeft);
        super.backRight.setPower(backRight);
    }

    public void setPowerDriveMotors(float leftSpeed, float rightSpeed) {
        frontLeft.setPower(Range.clip(leftSpeed, -1, 1));
        backLeft.setPower(Range.clip(leftSpeed, -1, 1));
        frontRight.setPower(Range.clip(rightSpeed, -1, 1));
        backRight.setPower(Range.clip(rightSpeed, -1, 1));
    }

    public void setPowerDriveMotors(double power) {
        for (DcMotor motor : driveMotors) {
            motor.setPower(power);
        }
    }

    //-------------- MOVEMENT -------------- (organization)


    // Takes in unit circle coordinates and rotation speed and outputs motor powers to put into the motors
    // The robot's speed/max speed in the direction will be the magnitude of the coordinates
    public double[] circle_to_taxicab(double circle_x, double circle_y, double circle_rotate) {
        double[] answer = new double[4];
        double x;
        double y;

        if (circle_x == 0.0) {
            x = 0.0;
        } else {
            x = circle_x / Math.abs(circle_x) * Math.sqrt(Math.pow(circle_x, 2) + Math.pow(circle_y, 2))
                    * (Math.abs(circle_x)) / (Math.abs(circle_x) + Math.abs(circle_y));
        }
        if (circle_y == 0.0) {
            y = 0.0;
        } else {
            y = circle_y / Math.abs(circle_y) * Math.sqrt(Math.pow(circle_x, 2) + Math.pow(circle_y, 2))
                    * (Math.abs(circle_y)) / (Math.abs(circle_x) + Math.abs(circle_y));
        }
        double sum = Math.abs(x) + Math.abs(y) + Math.abs(circle_rotate);
        if (sum > 1) {
            answer[0] = (x + y + circle_rotate) / sum;
            answer[1] = (-x + y + circle_rotate) / sum;
            answer[2] = (x + y - circle_rotate) / sum;
            answer[3] = (-x + y - circle_rotate) / sum;
        } else {
            answer[0] = (x + y + circle_rotate);
            answer[1] = (-x + y + circle_rotate);
            answer[2] = (x + y - circle_rotate);
            answer[3] = (-x + y - circle_rotate);
        }
        return answer;
    }


    public static final double DEFAULT_MOVE_TOLERANCE = 1.5; // SET DEFAULT TOLERANCE HERE
    //public Position currentPosition = new Position(0, 0);

    public double currentX;
    public double currentY;

    public double targetX;
    public double targetY;

   /* public void moveTo(double x, double y) {  //if red,positions are the bottom right of robot
                                                //if blue, positions are the bottom left of robot
    public void moveTo(double x, double y) {  //if red,positions are the bottom right of robot
        //if blue, positions are the bottom left of robot
        double currentAngle = getAngle();
        double currentX = currentPosition.getX();
        double currentY = currentPosition.getY();
        double magnitude = Math.sqrt(Math.pow(x - currentX, 2) + Math.pow(y - currentY, 2));
        //if currentAngle is the direction of the front of the robot, with the horizontal drawn from the robot:
        //angleOfPosition is the direction of the moveTO position with the horizontal drawn from the robot
        //x < currentX: if atan needs to be correct
        double angleOfPosition;
        if (currentX == x) {
            if (y < currentY) {
                angleOfPosition = 3 * Math.PI / 2;
            } else {
                angleOfPosition = Math.PI / 2;
            }
        } else {
            angleOfPosition = Math.atan((y - currentY) / (x - currentX)) + (x < currentX ? Math.PI : 0);
        }
        //look at the image if you want explanation
        double difference = currentAngle - angleOfPosition;
        double moveX = magnitude * Math.sin(difference);
        double moveY = magnitude * Math.cos(difference);

        move(moveX, moveY);
        //TODO: temporary
        currentPosition = new Position(x, y);
    }*/

    public void setTargetPosition(double x, double y) {
        targetX = x;
        targetY = y;
        //WORK HERE ERIC
        double angle = getRotationInDimension('Z');
        double[] translatedCoord = rotatePoint(targetX - currentX, targetY - currentY, angle);
        move(translatedCoord[0], translatedCoord[1]);

    }

    public void moveRelative(double x, double y) {//a move method relative to the robot, but with the coordinate plane
        setTargetPosition(targetX + x, targetY + y);
    }

    //todo: make moveto / coordinates and shit
    public void move(double inchesSideways, double inchesForward) {
        move(inchesSideways, inchesForward, 1);
    }

    public void moveAddTolerance(double inchesSideways, double inchesForward, double maxPower, double inchesToleranceAddition) {
        move(inchesSideways, inchesForward, DEFAULT_MOVE_TOLERANCE + inchesToleranceAddition);
    }

    public void move(double inchesSideways, double inchesForward, double inchesTolerance) {
        double p = -0.000055;
        double i = -0.00001;
        double d = 0.000011;

    public void move(double inchesSideways, double inchesForward, double inchesTolerance) {
        double p = -0.00005;
        double i = -0.000012;
        double d = 0.000012;
        move(inchesSideways, inchesForward, p, i, d, inchesTolerance);
    }

    private static double[] rotatePoint(double x, double y, double angle /*in degrees*/) {
        double[] translatedPoint = new double[2];
        double angleRad = Math.PI / 180 * angle;
        translatedPoint[0] = x * Math.cos(angleRad) - y * Math.sin(angleRad);
        translatedPoint[1] = y * Math.cos(angleRad) + x * Math.sin(angleRad);
        return translatedPoint;
    }

    /*
    public void move(double inchesSideways, double inchesForward, double Kp, double Ki, double Kd, double inchesTolerance) {
        inchesSideways = -inchesSideways;
        odoWheelSideways.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        odoWheelForwards.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        odoWheelSideways.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        odoWheelForwards.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double toleranceEncoder = convertInchToEncoderOdom(inchesTolerance);
        double xTarget = convertInchToEncoderOdom(inchesSideways);
        double yTarget = convertInchToEncoderOdom(inchesForward);
        double minSpeed = 0;// 0.005; // Change later

        double disFront = yTarget; // Y displacement from the target in encoders
        double disSide = xTarget; // X displacement from the target in encoders
        double speedFront = 0; // Y speed in encoders per second
        double speedSide = 0; // X speed in encoders per second
        double areaFront = 0; // Y area in encoders * seconds
        double areaSide = 0; // X area in encoders * seconds

        double frontOdometryLastPosition = odoWheelForwards.getCurrentPosition();
        double sidewaysOdometryLastPosition = odoWheelSideways.getCurrentPosition();

        double pRot = 0.02; // Rotate p control loop for
        // toleranceRot, won't rotate the robot if within the tolerance. If the
        // angle is within the tolerance but later isn't the rotate p control loop will activate again
        double toleranceRot = 5;

        // variables to help determine orientation
        double initialAngle = getRotationInDimension('Z'); // -180 to 180
        double lastAngleIMU = initialAngle; // -180 to 180
        double currentAngle = initialAngle; // -inf to inf, clockwise

        // ElapsedTime interval is the time between consecutive loops
        // ElapsedTime is the time of the method itself
        ElapsedTime interval = new ElapsedTime();
        ElapsedTime time = new ElapsedTime();

        while (opModeIsActive() && (Math.sqrt(Math.pow(disFront, 2) + Math.pow(disSide, 2))) > toleranceEncoder) {
            // updates current angle
            // The normal imu angle is a problem because it only goes from -180 to 180, so the
            // change in angle would be off sometimes if you subtracted two imu angles
            double changeInAngle;
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

            // Updates the area, displacement, and speed variables for the PID loop
            disFront = yTarget - odoWheelForwards.getCurrentPosition();
            disSide = xTarget - odoWheelSideways.getCurrentPosition();
            speedFront = (odoWheelForwards.getCurrentPosition() - frontOdometryLastPosition) / interval.seconds();
            speedSide = (odoWheelSideways.getCurrentPosition() - sidewaysOdometryLastPosition) / interval.seconds();
            areaFront += interval.seconds() * disFront;
            areaSide += interval.seconds() * disSide;

            // Cannot start at any power faster than .5 because the rotation slip would be very high
            // Gradually increases the maxPower
            double maxPower = Math.min(.5 + .5 * time.seconds(), 1);

            double magnitude = Math.sqrt(Math.pow(disSide, 2) + Math.pow(disFront, 2));
            double[] motorPowers = circle_to_taxicab(-disSide / magnitude, -disFront / magnitude, 0);


            // Sets the actual motor powers according to PID
            // Clips it so the motor power is not too low to avoid steady-state or goes too fast
            if (Math.abs(currentAngle - initialAngle) > toleranceRot) {
                // Angle is not within tolerance, corrects using rotation p control loop as the robot strafes
                frontLeft.setPower(motorClip(pRot * (currentAngle - initialAngle) + Kp * (disFront - disSide) + Ki * (areaFront - areaSide) + Kd * (speedFront - speedSide), minSpeed, maxPower));
                backRight.setPower(motorClip(-pRot * (currentAngle - initialAngle) + Kp * (disFront - disSide) + Ki * (areaFront - areaSide) + Kd * (speedFront - speedSide), minSpeed, maxPower));
                frontRight.setPower(motorClip(-pRot * (currentAngle - initialAngle) + Kp * (disFront + disSide) + Ki * (areaFront + areaSide) + Kd * (speedFront + speedSide), minSpeed, maxPower));
                backLeft.setPower(motorClip(pRot * (currentAngle - initialAngle) + Kp * (disFront + disSide) + Ki * (areaFront + areaSide) + Kd * (speedFront + speedSide), minSpeed, maxPower));
            } else {
                // Angle is in the angle tolerance, does not activate rotation p control loop
                frontLeft.setPower(motorClip(Kp * (disFront - disSide) + Ki * (areaFront - areaSide) + Kd * (speedFront - speedSide), minSpeed, maxPower));
                backRight.setPower(motorClip(Kp * (disFront - disSide) + Ki * (areaFront - areaSide) + Kd * (speedFront - speedSide), minSpeed, maxPower));
                frontRight.setPower(motorClip(Kp * (disFront + disSide) + Ki * (areaFront + areaSide) + Kd * (speedFront + speedSide), minSpeed, maxPower));
                backLeft.setPower(motorClip(Kp * (disFront + disSide) + Ki * (areaFront + areaSide) + Kd * (speedFront + speedSide), minSpeed, maxPower));

            double maxPower = Math.min(.5 + .5 * time.seconds(), 1);
            if (Math.abs(currentAngle - initialAngle) > 3) {
                frontLeft.setPower(motorClip(-pRot * (currentAngle - initialAngle) + motorPowers[3], minSpeed, maxPower));
                backRight.setPower(motorClip(pRot * (currentAngle - initialAngle) + motorPowers[1], minSpeed, maxPower));
                frontRight.setPower(motorClip(pRot * (currentAngle - initialAngle) + motorPowers[0], minSpeed, maxPower));
                backLeft.setPower(motorClip(-pRot * (currentAngle - initialAngle) + motorPowers[2], minSpeed, maxPower));
            } else {
                frontRight.setPower(motorPowers[0]);
                backRight.setPower(motorPowers[1]);
                backLeft.setPower(motorPowers[2]);
                frontLeft.setPower(motorPowers[3]);
            }

            telemetry.addData("Motor Front Left and Back Right", frontLeft.getPower());
            telemetry.addData("Motor Front Right and Back Left", frontRight.getPower());
            telemetry.update();

            frontOdometryLastPosition = odoWheelForwards.getCurrentPosition();
            sidewaysOdometryLastPosition = odoWheelSideways.getCurrentPosition();
            interval.reset();
        }
        //turnAbsolute(initialAngle);
        setPowerDriveMotors(0);
        targetX += rotatePoint(xTarget - disSide, yTarget - disFront, getRotationInDimension('Z'))[0];
        targetY += rotatePoint(xTarget - disSide, yTarget - disFront, getRotationInDimension('Z'))[1];
    }

    private double motorClip(double power, double minPower, double maxPower) {
        if (power < 0) {
    */
    private static double[] rotatePoint(double x, double y, double angle /*in degrees*/) {
        double[] translatedPoint = new double[2];
        double angleRad = Math.PI / 180 * angle;
        translatedPoint[0] = x * Math.cos(angleRad) - y * Math.sin(angleRad);
        translatedPoint[1] = y * Math.cos(angleRad) + x * Math.sin(angleRad);
        return translatedPoint;
    }

    public void move(double inchesSideways, double inchesForward, double Kp, double Ki, double Kd, double inchesTolerance) {
        final double Odometry_Sideways_Error = -15.8524; // encoders per degree
        final double Odometry_Forwards_Error = -167.92024; // encoders per degree

        inchesSideways = -inchesSideways;
        odoWheelSideways.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        odoWheelForwards.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        odoWheelSideways.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        odoWheelForwards.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double toleranceEncoder = convertInchToEncoderOdom(inchesTolerance);
        double xTarget = convertInchToEncoderOdom(inchesSideways);
        double yTarget = convertInchToEncoderOdom(inchesForward);
        double minSpeed = 0;// 0.005; // Change later

        double disFront = yTarget; // Y displacement from the target in encoders
        double disSide = xTarget; // X displacement from the target in encoders

        double xRobot = 0; // X coordinate
        double yRobot = 0; // Y coordinate
        double speedFront = 0; // Y speed in encoders per second
        double speedSide = 0; // X speed in encoders per second
        double areaFront = 0; // Y area in encoders * seconds
        double areaSide = 0; // X area in encoders * seconds

        double frontOdometryLastPosition = odoWheelForwards.getCurrentPosition();
        double sidewaysOdometryLastPosition = odoWheelSideways.getCurrentPosition();

        double pRot = 0.01; // Rotate p control loop for
        // toleranceRot, won't rotate the robot if within the tolerance. If the
        // angle is within the tolerance but later isn't the rotate p control loop will activate again
        double toleranceRot = 2;
        double iRot = .01;

        // variables to help determine orientation
        double initialAngle = getRotationInDimension('Z'); // -180 to 180
        double lastAngleIMU = initialAngle; // -180 to 180
        double currentAngle = initialAngle; // -inf to inf, clockwise

        // ElapsedTime interval is the time between consecutive loops
        // ElapsedTime is the time of the method itself
        ElapsedTime interval = new ElapsedTime();
        ElapsedTime time = new ElapsedTime();

        double angleArea = 0;
        while (opModeIsActive() && ((Math.sqrt(Math.pow(disFront, 2) + Math.pow(disSide, 2))) > toleranceEncoder || Math.abs(currentAngle - initialAngle) > toleranceRot)) {
            // updates current angle
            // The normal imu angle is a problem because it only goes from -180 to 180, so the
            // change in angle would be off sometimes if you subtracted two imu angles
            double changeInAngle;
            double currentAngleIMU = getRotationInDimension('Z');
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
            double frontOdometryWheelCurrentPosition = odoWheelForwards.getCurrentPosition();
            double sidewaysOdometryWheelCurrentPosition = odoWheelSideways.getCurrentPosition();
            double intervalTime = interval.seconds();
            interval.reset();
            double[] rotatedDisplacement = rotatePoint(sidewaysOdometryWheelCurrentPosition - sidewaysOdometryLastPosition - changeInAngle * Odometry_Sideways_Error, frontOdometryWheelCurrentPosition - frontOdometryLastPosition - changeInAngle * Odometry_Forwards_Error, currentAngle - initialAngle);
            // Updates the area, displacement, and speed variables for the PID loop
            yRobot += rotatedDisplacement[1];
            xRobot += rotatedDisplacement[0];
            speedFront = rotatedDisplacement[1] / intervalTime;
            speedSide = rotatedDisplacement[0] / intervalTime;
            disFront = yTarget - yRobot;
            disSide = xTarget - xRobot;
            areaFront += intervalTime * disFront;
            areaSide += intervalTime * disSide;

            frontOdometryLastPosition = frontOdometryWheelCurrentPosition;
            sidewaysOdometryLastPosition = sidewaysOdometryWheelCurrentPosition;

            // Cannot start at any power faster than .5 because the rotation slip would be very high
            // Gradually increases the maxPower
            double maxPower = Math.min(.5 + .5 * time.seconds(), .9);

            double[] motorsStuff = {Kp * (disFront - disSide) + Ki * (areaFront - areaSide) + Kd * (speedFront - speedSide), Kp * (disFront + disSide) + Ki * (areaFront + areaSide) + Kd * (speedFront + speedSide)};
            motorsStuff = rotatePoint(motorsStuff[0], motorsStuff[1], initialAngle - currentAngle);

            // Sets the actual motor powers according to PID
            // Clips it so the motor power is not too low to avoid steady-state or goes too fast

            if (Math.abs(currentAngle - initialAngle) > toleranceRot) {
                // Angle is not within tolerance, corrects using rotation p control loop as the robot strafes
                frontLeft.setPower(-iRot * angleArea + -pRot * (currentAngle - initialAngle) + motorClip(motorsStuff[0], minSpeed, maxPower));
                backRight.setPower(iRot * angleArea + pRot * (currentAngle - initialAngle) + motorClip(motorsStuff[0], minSpeed, maxPower));
                frontRight.setPower(iRot * angleArea + pRot * (currentAngle - initialAngle) + motorClip(motorsStuff[1], minSpeed, maxPower));
                backLeft.setPower(-iRot * angleArea - pRot * (currentAngle - initialAngle) + motorClip(motorsStuff[1], minSpeed, maxPower));
                angleArea += intervalTime * (currentAngle - initialAngle);
            } else {
                // Angle is in the angle tolerance, does not activate rotation p control loop
                frontLeft.setPower(motorClip(motorsStuff[0], minSpeed, maxPower));
                backRight.setPower(motorClip(motorsStuff[0], minSpeed, maxPower));
                frontRight.setPower(motorClip(motorsStuff[1], minSpeed, maxPower));
                backLeft.setPower(motorClip(motorsStuff[1], minSpeed, maxPower));
                angleArea = 0;
            }
            double[] answer = circle_to_taxicab(gamepad1.left_stick_x, gamepad1.left_stick_y, ROTATE_TO_MOVE_RATIO * gamepad1.right_stick_x);
            double factor = 1;
            if (gamepad1.left_bumper) {
                // if left bumper is pressed, reduce the motor speed
                factor = TeleOpMethods.LEFT_BUMPER_TRIGGER_FACTOR;
            }
            if (gamepad1.right_bumper) {
                // if right bumper is pressed, reduce the motor speed
                factor = TeleOpMethods.RIGHT_BUMPER_TRIGGER_FACTOR;
            }
            /*
            frontRight.setPower(factor * answer[0]);
            backRight.setPower(factor * answer[1]);
            backLeft.setPower(factor * answer[2]);
            frontLeft.setPower(factor * answer[3]);*/

            telemetry.addData("xRobot", convertEncoderToInchOdom(xRobot));
            telemetry.addData("yRobot", convertEncoderToInchOdom(yRobot));
            telemetry.addData("front encoder", convertEncoderToInchOdom(odoWheelForwards.getCurrentPosition()));
            telemetry.addData("side encoder ", convertEncoderToInchOdom(odoWheelSideways.getCurrentPosition()));

            telemetry.addData("CurrentAngle", currentAngle);
            telemetry.update();

        }
        setPowerDriveMotors(0);
    }

    private double motorClip(double power, double minPower, double maxPower) {
        if (power < 0) {
            return Range.clip(power, -maxPower, -minPower);
        } else if (power > 0) {
            return Range.clip(power, minPower, maxPower);
        } else {
            return 0;
        }
    }

    public static double getTransformedDistance(double inches) {  // leave this as a separate method, ENCAPSULATION ! !
        double m = .866279;
        double b = .0775;

        // inverse of equation y = mx + b: y = (1/m)(x - b)
        return (1 / m) * (inches - b);
    }

    //-------------- ROTATION -------------- (organization)

    public void turnRelative(double targetAngle) {
        turnAbsolute(getRotationInDimension('Z') + targetAngle);
    }

    public static final double DEFAULT_ROTATE_TOLERANCE = 5; // SET DEFAULT ROTATE TOLERANCE HERE

    public void turnAbsolute(double targetAngle) {
        // SAME AS THE OTHER MOVE METHOD, change the default value for tolerance here
        turnAbsolute(targetAngle, DEFAULT_ROTATE_TOLERANCE);
    }

    public void turnAbsoluteAddTolerance(double targetAngle, double toleranceAddition) {
        turnAbsolute(targetAngle, DEFAULT_ROTATE_TOLERANCE + toleranceAddition);
    }

    public void turnAbsolute(double targetAngle, double inputTolerance) {
        double currentAngle;
        int direction;
        double turnRate;
        double P = 0.02; //set later
        double tolerance = inputTolerance; //set later
        double maxSpeed = 0.4; //set later
        double minSpeed = 0.01; //set later
        double error;

        do {
            currentAngle = getRotationInDimension('Z');
            error = getAngleDist(targetAngle, currentAngle);
            direction = getAngleDir(targetAngle, currentAngle);
            turnRate = Range.clip(P * error, minSpeed, maxSpeed);
            //telemetry.addData("error", error);
            //telemetry.addData("turnRate", turnRate);
            //telemetry.update();
            setPowerDriveMotors(-(float) (turnRate * direction), (float) (turnRate * direction));
        }
        while (opModeIsActive() && error > tolerance);
        setPowerDriveMotors(0);
    }

    //-------------- SLIDES -------------- (organization)

    public void setSlidePower(double power) {
        slideUp.setPower(power);
        slideDown.setPower(power);
    }

    public void extendSlidesTo(int encoderPos, double power) {
        if (encoderPos > slideMax)
            encoderPos = slideMax;
        else if (encoderPos < slideMin)
            encoderPos = slideMin;

        slideUp.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideDown.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideUp.setTargetPosition(encoderPos);
        slideDown.setTargetPosition(encoderPos);
        slideUp.setPower(power);
        slideDown.setPower(power);
    }


    public void extendSlidesBy(double inches, double speed) {

        int position = (slideUp.getCurrentPosition() + slideDown.getCurrentPosition()) / 2;
        int encoderVal = convertInchToEncoderSlides(inches);

        int val = position + encoderVal;

        slideUp.setTargetPosition(val);
        slideDown.setTargetPosition(val);
        slideUp.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideDown.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideDown.setPower(speed);
        slideUp.setPower(speed);
    }

    //-------------- GADGETS  -------------- (organization)

    //Negative = backwards, positive = forwards
    public void setIntakeWheels(double power) {
        intakeLeft.setPower(power);
        intakeRight.setPower(power);
    }

    //hook for moving foundation, true = down, false = up
    public void toggleHook(boolean down) {
        //might need to recalibrate
        if (down) {
            hookServoLeft.setPosition(.44);
            hookServoRight.setPosition(.48);
        } else {
            hookServoLeft.setPosition(0.1);
            hookServoRight.setPosition(0.1);
        }
        // sleep(500);
    }

    //TODO values need to be changed
    public void toggleHood(boolean down) {
        if (down)
            hoodServo.setPosition(1);
        else
            hoodServo.setPosition(0);
    }

    public void setIntakeServosPosition(double position) {
        intakeServoLeft.setPosition(position);
        intakeServoRight.setPosition(position);
    }

    //yeeter methods
    public void extendYeeter() { //extend yeeter to park
        //set position later
        yeetServo.setPosition(0.5);
    }

    //-------------- AUTO AID METHODS  -------------- (organization)

    public void releaseStone() { //Servos to open position
        intakeServoLeft.setPosition(.55);

        intakeServoRight.setPosition(.85);
        setIntakeWheels(0);
    }

    public void outwardsIntake() { //moves intake servos all the way open
        intakeServoLeft.setPosition(.3);
        intakeServoRight.setPosition(.6);
    }

    public void intakeStone() { //servos to a position to open, turns on intake wheels
        setIntakeWheels(0.9);
        intakeServoLeft.setPosition(.6);
        intakeServoRight.setPosition(.9);
    }

    public void clampStone() { //servos to close position
        intakeServoLeft.setPosition(.67);
        intakeServoRight.setPosition(.97);
    }

    public void stopIntakeWheels() {
        setIntakeWheels(0);
    }


    public void bringSlidesDown() {
        extendSlidesBy(2, 0.5);
        sleep(500);
        releaseStone();
        sleep(500);
        extendSlidesTo(slideMin, 0.5);
    }

    public void yuhwanSlidesDown() {
        extendSlidesBy(2, 0.5);
        sleep(500);
        outwardsIntake();
        sleep(500);
        extendSlidesTo(slideMin, .5);
    }

    public void yuhwanIntakeStone() {
        clampStone();
        setIntakeWheels(.7);
        sleep(1000);
        stopIntakeWheels();
    }

    public int grabSkystone(String color) {
        int factor;
        if (color.equals("red"))
            factor = 1;
        else
            factor = -1;
        turnRelative(factor * 45);

        boolean center = false;

        if (tfod != null) {
            tfod.activate();
        }

        waitForStart();

        ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        timer.reset();
        //timer.startTime();

        int blocksMoved = 0;
        if (opModeIsActive()) {
            if (tfod != null) {
                while (!center && blocksMoved < 3) {  // should move 3 blocks at max, otherwise vision doesn't work, move on
                    moveAddTolerance(factor * Math.sqrt(2) * 4, -Math.sqrt(2) * 4, .7, 1);
                    //List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    List<Recognition> updatedRecognitions = tfod.getRecognitions();

                    blocksMoved++;
                    //sleep(500);
                    while (timer.time() < 3000) {
                        telemetry.addData("timer: %d", timer.time());

                        if (updatedRecognitions != null) {
                            for (Recognition recognition : updatedRecognitions) {
                                telemetry.addData(String.format("label (%d)", 0), recognition.getLabel());
                                if (recognition.getLabel().equals(LABEL_SECOND_ELEMENT)) {
                                    float stonePos = (recognition.getRight() + recognition.getLeft()) / 2;  // vertical phone im pretty sure

                                    //float stonePos = (recognition.getTop() + recognition.getBottom()) / 2;
                                    center = TOP_MIDDLE_SCREEN + 160 > stonePos && stonePos > TOP_MIDDLE_SCREEN - 160;
                                    //center = stonePos + 160 > TOP_MIDDLE_SCREEN && TOP_MIDDLE_SCREEN > stonePos - 160;
                                }
                            }
                        }
                        telemetry.update();

                    }
                    timer.reset();
                }
            }
            // setPowerDriveMotors(0);
        }

        //move forward, grab block, move back
        intakeStone();

        move(0, 14, 0.3);
        clampStone();

        sleep(250);
        intakeStone();
        clampStone();
        sleep(250);
        stopIntakeWheels();
        sleep(250);
        move(0, -14, 0.3);

        //turns back
        turnRelative(factor * -45);

        return blocksMoved * 8;
    }

}