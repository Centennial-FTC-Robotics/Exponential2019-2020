package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

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
    public void setStartingAngle(double start) {
        decay.startingAngle = start;
    }

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
    public double[] getMotorPowers(double triggerX, double triggerY, double rotate) {
        double[] motorPowers = new double[4];
        double x;
        double y;

        if (triggerX == 0.0) {
            x = 0.0;
        } else {
            x = triggerX / Math.abs(triggerX) * Math.sqrt(Math.pow(triggerX, 2) + Math.pow(triggerY, 2))
                    * (Math.abs(triggerX)) / (Math.abs(triggerX) + Math.abs(triggerY));
        }
        if (triggerY == 0.0) {
            y = 0.0;
        } else {
            y = triggerY / Math.abs(triggerY) * Math.sqrt(Math.pow(triggerX, 2) + Math.pow(triggerY, 2))
                    * (Math.abs(triggerY)) / (Math.abs(triggerX) + Math.abs(triggerY));
        }

        double sum = Math.abs(x) + Math.abs(y) + Math.abs(rotate);
        if (sum > 1) {
            motorPowers[0] = (x + y - rotate) / sum;
            motorPowers[1] = (-x + y - rotate) / sum;
            motorPowers[2] = (x + y + rotate) / sum;
            motorPowers[3] = (-x + y + rotate) / sum;
        } else {
            motorPowers[0] = (x + y - rotate);
            motorPowers[1] = (-x + y - rotate);
            motorPowers[2] = (x + y + rotate);
            motorPowers[3] = (-x + y + rotate);
        }
        return motorPowers;
    }


    public static final double DEFAULT_MOVE_TOLERANCE = 1.5; // SET DEFAULT TOLERANCE HERE
    //public Position currentPosition = new Position(0, 0);


    // implements Runnable for multiThreading if needed
    class ExpoBot implements Runnable {
        final static double Odometry_Forwards_Error = -191.92024; // encoders per degree
        final static double Odometry_Sideways_Error = -21.8524; // encoders per degree

        // The following 6 variables are in terms of the field
        // In terms of encoders too
        public double currentX = 0;
        public double currentY = 0;
        public double velX = 0;
        public double velY = 0;
        public double targetX = 0;
        public double targetY = 0;

        // positive X on the field is angle = 0
        // positive goes counterclockwise
        // angles are unnormalized
        public double startingAngle = 90;
        public double targetAngle = startingAngle;
        public double currentAngle = startingAngle;
        public double angleVel = 0;

        public void setTarget(double targetX, double targetY, double targetAngle) {
            this.targetX = targetX;
            this.targetY = targetY;
            this.targetAngle = targetAngle;
        }

        public void run() {
            while(opModeIsActive()) {
                updateRobot();
            }
        }

        // variables it needs to keep track of position with
        private double lastAngleIMU = getRotationInDimension('Z'); // -180 to 180
        private double changeInAngle = 0;
        private double frontOdometryLastPosition = odoWheelForwards.getCurrentPosition();
        private double sidewaysOdometryLastPosition = odoWheelSideways.getCurrentPosition();


        private ElapsedTime time = new ElapsedTime();

        // Try to call this method at least once every 0.1 milliseconds
        public void updateRobot() {

            double frontOdometryWheelCurrentPosition = odoWheelForwards.getCurrentPosition();
            double sidewaysOdometryWheelCurrentPosition = odoWheelSideways.getCurrentPosition();

            // changes the current angle (still unnormalized)
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
            angleVel = changeInAngle / time.seconds();

            // in terms of the robot
            double[] displacement = {sidewaysOdometryWheelCurrentPosition - sidewaysOdometryLastPosition - changeInAngle * Odometry_Sideways_Error,
                    frontOdometryWheelCurrentPosition - frontOdometryLastPosition - changeInAngle * Odometry_Forwards_Error};

            // rotates the displacement vector so its in terms of the field
            displacement = rotatePoint(displacement[0], displacement[1], currentAngle - 90);

            currentX += displacement[0];
            currentY += displacement[1];
            double timerInterval = time.seconds();
            time.reset();
            velX = displacement[0] / timerInterval;
            velY = displacement[1] / timerInterval;

            frontOdometryLastPosition = frontOdometryWheelCurrentPosition;
            sidewaysOdometryLastPosition = sidewaysOdometryWheelCurrentPosition;
        }
    }

    public ExpoBot decay = new ExpoBot();


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
    }*/

    public void setTargetAngle(double angle) {
        decay.targetAngle = angle;
    }

    public void setTargetPosition(double x, double y, double targetAngle) {
        decay.targetX = x;
        decay.targetY = y;
        //WORK HERE ERIC
        double angle = getRotationInDimension('Z');
        double[] translatedCoord = rotatePoint(decay.targetX - decay.currentX, decay.targetY - decay.currentY, angle);
        move(translatedCoord[0], translatedCoord[1]);
    }

    public void setTargetPosition(double x, double y) {
        decay.targetX = x;
        decay.targetY = y;
        //WORK HERE ERIC
        double angle = getRotationInDimension('Z');
        double[] translatedCoord = rotatePoint(decay.targetX - decay.currentX, decay.targetY - decay.currentY, -angle);
        move(translatedCoord[0], translatedCoord[1]);
    }

    public void moveRelative(double x, double y, double angle) {
        System.out.println("YEAH");
    }

    public void moveRelative(double x, double y) {//a move method relative to the robot, but with the coordinate plane
        setTargetPosition(decay.targetX + x, decay.targetY + y);
    }

    //todo: make moveto / coordinates and shit
    public void move(double inchesSideways, double inchesForward) {
        move(inchesSideways, inchesForward, 1);
    }

    public void moveAddTolerance(double inchesSideways, double inchesForward, double maxPower, double inchesToleranceAddition) {
        move(inchesSideways, inchesForward, DEFAULT_MOVE_TOLERANCE + inchesToleranceAddition);
    }

    public void move(double inchesSideways, double inchesForward, double inchesTolerance) {
        double p = 0.00005;
        double i = 0.0000135;
        double d = 0.000014;
        // TODO: 3/18/2020 Make sure Diego's modification to the odometry wheels doesn't change anything (no wrong direction on either and change encoders per radian)
        // TODO: fix all the PID stuff because now everything is inverted (including rotation and linear)
        // PID numbers (I know, they're supposed to be all positive, I think that the motors maybe in the wrong direction)

        move(inchesSideways, inchesForward, p, i, d, inchesTolerance);
    }

    private static double[] rotatePoint(double x, double y, double angle /*in degrees*/) {
        double[] translatedPoint = new double[2];
        double angleRad = Math.PI / 180 * angle;
        translatedPoint[0] = x * Math.cos(angleRad) - y * Math.sin(angleRad);
        translatedPoint[1] = y * Math.cos(angleRad) + x * Math.sin(angleRad);
        return translatedPoint;
    }

    public void move(double inchesSideways, double inchesForward, double Kp, double Ki, double Kd, double inchesTolerance) {
        moveWithAngle(inchesSideways, inchesForward, Kp, Ki, Kd, inchesTolerance, decay.targetAngle);
    }


    public void moveWithAngle(double inchesSideways, double inchesForward, double Kp, double Ki, double Kd, double inchesTolerance, double targetAngle) {
        decay.targetAngle = targetAngle;
        double[] changeTargetBy = {inchesSideways, inchesForward};

        decay.updateRobot();
        changeTargetBy = rotatePoint(changeTargetBy[0], changeTargetBy[1], decay.currentAngle - 90);
        decay.targetX += convertInchToEncoderOdom(changeTargetBy[0]);
        decay.targetY += convertInchToEncoderOdom(changeTargetBy[1]);
        runToTarget(inchesTolerance, Kp, Ki, Kd);
    }


    // Tells the robot to move to the target
    public void runToTarget(double inchesTolerance, double Kp, double Ki, double Kd) {
        frontRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // Displacement for p portion of the PID loop
        double xDis = decay.targetX - decay.currentX;
        double yDis = decay.targetY - decay.currentY;
        double angleDis = decay.targetAngle - decay.currentAngle;

        // Area for i portion of the PID loop
        double xArea = 0;
        double yArea = 0;
        double angleArea = 0;

        // Velocity for d portion of the PID loop
        double xVel = 0;
        double yVel = 0;
        double angleVel = 0;

        ElapsedTime whileLoopTime = new ElapsedTime();
        // The motor power portion for the linear movement (there will also be a rotational movement)
        // 0th index is frontRight and backLeft, 1th index is frontLeft and backRight
        double[] linearMotorPowers = new double[2];

        // variables for angle correction PID
        double angleTolerance = 5;
        double angleP = .02;
        double angleI = .01;
        double angleD = 0;

        // maxPower for linear movement
        double maxPower = 0.8;

        while (opModeIsActive() && !within(inchesTolerance)) {
            linearMotorPowers[0] = Kp * yDis + Ki * yArea - Kd * yVel + Kp * xDis + Ki * xArea - Kd * xVel;
            linearMotorPowers[1] = Kp * yDis + Ki * yArea - Kd * yVel - (Kp * xDis + Ki * xArea - Kd * xVel);

            // rotates the linearMotorsPowers to get them in terms of robot
            linearMotorPowers = rotatePoint(linearMotorPowers[0], linearMotorPowers[1], -decay.currentAngle);
            linearMotorPowers[0] = Range.clip(linearMotorPowers[0], -maxPower, maxPower);
            linearMotorPowers[1] = Range.clip(linearMotorPowers[1], -maxPower, maxPower);

            if (Math.abs(angleDis) > angleTolerance) {
                frontRight.setPower(angleP * angleDis + angleI * angleArea - angleD * angleVel + linearMotorPowers[0]);
                frontLeft.setPower(-angleP * angleDis - angleI * angleArea + angleD * angleVel + linearMotorPowers[1]);
                backRight.setPower(angleP * angleDis + angleI * angleArea - angleD * angleVel + linearMotorPowers[1]);
                backLeft.setPower(-angleP * angleDis - angleI * angleArea + angleD * angleVel + linearMotorPowers[0]);
            } else {
                frontRight.setPower(linearMotorPowers[0]);
                frontLeft.setPower(linearMotorPowers[1]);
                backRight.setPower(linearMotorPowers[1]);
                backLeft.setPower(linearMotorPowers[0]);
            }
            decay.updateRobot();
            double time = whileLoopTime.seconds();
            whileLoopTime.reset();
            xDis = decay.targetX - decay.currentX;
            yDis = decay.targetY - decay.currentY;
            angleDis = decay.targetAngle - decay.currentAngle;
            if (within(5)) {
                xArea += xDis * time;
                yArea += yDis * time;
            } else {
                xArea = 0;
                yArea = 0;
            }
            if (Math.abs(angleDis) > angleTolerance) {
                angleArea += angleDis * time;
                angleArea = Range.clip(angleArea, -0.2, 0.2);
            } else {
                angleArea = 0;
            }
            xVel = decay.velX;
            yVel = decay.velY;
            angleVel = decay.angleVel;
        }
    }

    // returns whether the robot is within a certain radius of the target position
    // DOES NOT UPDATE THE ROBOT POSITION, you need to do that manually
    private boolean within(double radiusInch) {
        double xDis = decay.targetX - decay.currentX;
        double yDis = decay.targetY - decay.currentY;
        double radius = convertInchToEncoderOdom(radiusInch);
        return Math.sqrt(Math.pow(xDis, 2) + Math.pow(yDis, 2)) <= radius;
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
        decay.targetAngle = targetAngle;
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
        double P = 0.035; //set later
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
            setPowerDriveMotors((float) (turnRate * direction), -(float) (turnRate * direction));
        }
        while (opModeIsActive() && error > tolerance);
        setPowerDriveMotors(0);
        setTargetAngle(targetAngle);
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
            hookServoLeft.setPosition(1);
            hookServoRight.setPosition(1);
        } else {
            hookServoLeft.setPosition(0);
            hookServoRight.setPosition(0);
        }
        // sleep(500);
    }

    public void toggleClaw(boolean down) {

        if (down) {
            claw.setPosition(0);
        } else {
            claw.setPosition(1);
        }
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