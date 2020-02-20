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

import java.util.List;

import static org.firstinspires.ftc.robotcore.external.tfod.TfodRoverRuckus.TFOD_MODEL_ASSET;

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

    public static final int slideMax = 2250;
    public static final int slideMin = -218;

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

    public double getRotationinDimension(char dimension) {
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

    public static final double DEFAULT_MOVE_TOLERANCE = 1.5; // SET DEFAULT TOLERANCE HERE
    public static Position currentPosition = new Position(0, 0);
//todo: make moveto / coordinates and shit
    public void move(double inchesSideways, double inchesForward) {
        move(inchesSideways, inchesForward, MAX_POWER);
    }

    public void move(double inchesSideways, double inchesForward, double maxPower) {
        // this one just inputs the default value (.5) for tolerance
        move(inchesSideways, inchesForward, maxPower, DEFAULT_MOVE_TOLERANCE);
    }

    public void moveAddTolerance(double inchesSideways, double inchesForward, double maxPower, double inchesToleranceAddition) {
        move(inchesSideways, inchesForward, maxPower, DEFAULT_MOVE_TOLERANCE + inchesToleranceAddition);
    }
    /*
    public void move(double inchesSideways, double inchesForward, double maxPower, double inchesTolerance){
        double targetAngle = getRotationinDimension('Z');
        double currentAngle;
        int direction;
        double turnRate;
        // double P = 0.015; //set later
        double P = .02; //set later
        double maxSpeed = 1; //set later
        double minSpeed = 0.03; //set later
        double error;

        inchesForward = -inchesForward;

        double p = 1.0/800;
        double i;
        double d;
        //double inchesTolerance = .5;
        double max_positive = maxPower;
        double min_negative = -maxPower;

        double encoderForward = convertInchToEncoderOdom(inchesForward);
        double encoderSideways = convertInchToEncoderOdom(inchesSideways);
        odoWheelForwards.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        odoWheelSideways.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        odoWheelForwards.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        odoWheelSideways.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        int displacementSideways = (int)encoderSideways;
        int displacementForwards = (int)encoderForward;

        double tolerance = convertInchToEncoder(inchesTolerance);



        ElapsedTime time = new ElapsedTime();


        while (opModeIsActive()&&Math.abs(displacementSideways)>tolerance&&Math.abs(displacementForwards)>tolerance){
            currentAngle = getRotationinDimension('Z');

            error = getAngleDist(targetAngle, currentAngle);
            direction = getAngleDir(targetAngle, currentAngle);
            turnRate = Range.clip(P * error, minSpeed, maxSpeed);

            if(time.seconds()>.5){
                frontLeft.setPower(p*displacementForwards-p*displacementSideways);
                frontRight.setPower(p*displacementForwards+p*displacementSideways);
                backLeft.setPower(p*displacementForwards-p*displacementSideways);
                backRight.setPower(p*displacementForwards+p*displacementSideways);
            } else {
                frontLeft.setPower(-direction * turnRate + p*displacementForwards-p*displacementSideways);
                frontRight.setPower(direction * turnRate + p*displacementForwards+p*displacementSideways);
                backLeft.setPower(-direction * turnRate + p*displacementForwards-p*displacementSideways);
                backRight.setPower(direction * turnRate + p*displacementForwards+p*displacementSideways);
            }
            displacementSideways = (int)encoderSideways-odoWheelSideways.getCurrentPosition();
            displacementForwards = (int)encoderForward-odoWheelForwards.getCurrentPosition();
        }
        setPowerDriveMotors(0);
        //turnAbsolute(targetAngle);
    }
    */

    public void move(double inchesSideways, double inchesForward, double maxPower, double inchesTolerance) {  // DON'T FUCK WITH THIS METHOD, i will find a better way to do this later
        double p = 1.0 / 600;
        double i = 0.001;
        inchesForward = -inchesForward;
        inchesSideways = getTransformedDistance(inchesSideways);
        double d = 0;
        double tolerance = convertInchToEncoder(inchesTolerance);

        double frontLeft_encoder = convertInchToEncoder(inchesForward) - convertInchToEncoder(inchesSideways);
        double frontRight_encoder = convertInchToEncoder(inchesForward) + convertInchToEncoder(inchesSideways);
        double backLeft_encoder = convertInchToEncoder(inchesForward) + convertInchToEncoder(inchesSideways);
        double backRight_encoder = convertInchToEncoder(inchesForward) - convertInchToEncoder(inchesSideways);

        double areaFrontLeft = 0;
        double areaFrontRight = 0;
        double areaBackLeft = 0;
        double areaBackRight = 0;

        double frontLeftLastPosition = 0;
        double frontRightLastPosition = 0;
        double backLeftLastPosition = 0;
        double backRightLastPosition = 0;

        ElapsedTime time = new ElapsedTime();
        while (opModeIsActive() && (Math.abs(frontLeft_encoder - frontLeft.getCurrentPosition()) > tolerance || Math.abs(frontRight_encoder - frontRight.getCurrentPosition()) > tolerance || Math.abs(backLeft_encoder - backLeft.getCurrentPosition()) > tolerance || Math.abs(backRight_encoder - backRight.getCurrentPosition()) > tolerance)) {
            areaFrontLeft += time.seconds() * (frontLeft_encoder - frontLeft.getCurrentPosition());
            areaFrontRight += time.seconds() * (frontRight_encoder - frontRight.getCurrentPosition());
            areaBackLeft += time.seconds() * (backLeft_encoder - backLeft.getCurrentPosition());
            areaBackRight += time.seconds() * (backRight_encoder - backRight.getCurrentPosition());

            frontLeft.setPower(Range.clip(p * (frontLeft_encoder - frontLeft.getCurrentPosition()) + i * areaFrontLeft + d * (frontLeft.getCurrentPosition() - frontLeftLastPosition) / time.seconds(), -maxPower, maxPower));
            frontRight.setPower(Range.clip(p * (frontRight_encoder - frontRight.getCurrentPosition()) + i * areaFrontRight + d * (frontRight.getCurrentPosition() - frontRightLastPosition) / time.seconds(), -maxPower, maxPower));
            backLeft.setPower(Range.clip(p * (backLeft_encoder - backLeft.getCurrentPosition()) + i * areaBackLeft + d * (backLeft.getCurrentPosition() - backLeftLastPosition) / time.seconds(), -maxPower, maxPower));
            backRight.setPower(Range.clip(p * (backRight_encoder - backRight.getCurrentPosition()) + i * areaBackRight + d * (backRight.getCurrentPosition() - backRightLastPosition) / time.seconds(), -maxPower, maxPower));

            time.reset();
        }
        setPowerDriveMotors(0);
    }

    public void moveSetISetP(double inchesSideways, double inchesForward, double maxPower, double inchesTolerance, double i, double p) {  // DON'T FUCK WITH THIS METHOD, i will find a better way to do this later
        double targetAngle = getRotationinDimension('Z');
        inchesForward = -inchesForward;
        inchesSideways = getTransformedDistance(inchesSideways);

        if (inchesSideways != 0) {
            if (Math.sqrt(inchesSideways * inchesSideways + inchesForward * inchesForward) < 5) {
            } else {
            }
        } else {
            if (Math.sqrt(inchesSideways * inchesSideways + inchesForward * inchesForward) < 5) {
            } else {
            }
        }
        double d = 0;
        double max_positive = maxPower;
        double min_negative = -maxPower;

        double encoderForward = convertInchToEncoder(inchesForward);
        double encoderSideways = convertInchToEncoder(inchesSideways);
        resetDriveMotorEncoders();
        double tolerance = convertInchToEncoder(inchesTolerance);

        double frontLeft_encoder = encoderForward - encoderSideways;
        double frontRight_encoder = encoderForward + encoderSideways;
        double backLeft_encoder = encoderForward + encoderSideways;
        double backRight_encoder = encoderForward - encoderSideways;

        double frontLeft_displacement = frontLeft_encoder - frontLeft.getCurrentPosition();
        double frontRight_displacement = frontRight_encoder - frontRight.getCurrentPosition();
        double backLeft_displacement = backLeft_encoder - backLeft.getCurrentPosition();
        double backRight_displacement = backRight_encoder - backRight.getCurrentPosition();
        ElapsedTime time = new ElapsedTime();

        double areaFrontLeft = 0;
        double areaFrontRight = 0;
        double areaBackLeft = 0;
        double areaBackRight = 0;

        double frontLeftLastPosition = 0;
        double frontRightLastPosition = 0;
        double backLeftLastPosition = 0;
        double backRightLastPosition = 0;

        while (opModeIsActive() && (Math.abs(frontLeft_displacement) > tolerance || Math.abs(frontRight_displacement) > tolerance || Math.abs(backLeft_displacement) > tolerance || Math.abs(backRight_displacement) > tolerance)) {
            areaFrontLeft += time.seconds() * frontLeft_displacement;
            areaFrontRight += time.seconds() * frontRight_displacement;
            areaBackLeft += time.seconds() * backLeft_displacement;
            areaBackRight += time.seconds() * backRight_displacement;
            double speedFrontLeft = (frontLeft.getCurrentPosition() - frontLeftLastPosition) / time.seconds();
            double speedFrontRight = (frontRight.getCurrentPosition() - frontRightLastPosition) / time.seconds();
            double speedBackLeft = (backLeft.getCurrentPosition() - backLeftLastPosition) / time.seconds();
            double speedBackRight = (backRight.getCurrentPosition() - backRightLastPosition) / time.seconds();
            //if(time.seconds()>2){
            frontLeft.setPower(Range.clip(p * frontLeft_displacement + i * areaFrontLeft + d * speedFrontLeft, min_negative, max_positive));
            frontRight.setPower(Range.clip(p * frontRight_displacement + i * areaFrontRight + d * speedFrontRight, min_negative, max_positive));
            backLeft.setPower(Range.clip(p * backLeft_displacement + i * areaBackLeft + d * speedBackLeft, min_negative, max_positive));
            backRight.setPower(Range.clip(p * backRight_displacement + i * areaBackRight + d * speedBackRight, min_negative, max_positive));

            frontLeft_displacement = frontLeft_encoder - frontLeft.getCurrentPosition();
            frontRight_displacement = frontRight_encoder - frontRight.getCurrentPosition();
            backLeft_displacement = backLeft_encoder - backLeft.getCurrentPosition();
            backRight_displacement = backRight_encoder - backRight.getCurrentPosition();
            telemetry.addData("area frontLeft", areaFrontLeft);
            telemetry.addData("area frontRight", areaFrontRight);
            telemetry.addData("area backLeft", areaBackLeft);
            telemetry.addData("area backRight", areaBackRight);
            telemetry.update();
            time.reset();
        }
        setPowerDriveMotors(0);
        //TODO i got rid of rotate
        turnAbsolute(targetAngle);
    }


    public static double getTransformedDistance(double inches) {  // leave this as a separate method, ENCAPSULATION ! !
        double m = .866279;
        double b = .0775;

        // inverse of equation y = mx + b: y = (1/m)(x - b)
        return (1 / m) * (inches - b);
    }

    //-------------- ROTATION -------------- (organization)

    public void turnRelative(double targetAngle) {
        turnAbsolute(AngleUnit.normalizeDegrees(getRotationinDimension('Z') + targetAngle));
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
            currentAngle = getRotationinDimension('Z');
            error = getAngleDist(targetAngle, currentAngle);
            direction = getAngleDir(targetAngle, currentAngle);
            turnRate = Range.clip(P * error, minSpeed, maxSpeed);
            telemetry.addData("error", error);
            telemetry.addData("turnRate", turnRate);
            telemetry.update();
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
            hookServoLeft.setPosition(.45);
            hookServoRight.setPosition(.5);
        } else {
            hookServoLeft.setPosition(0.1);
            hookServoRight.setPosition(0.1);
        }
        // sleep(500);
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
        intakeServoRight.setPosition(.62);
    }

    public void intakeStone() { //servos to a position to open, turns on intake wheels
        setIntakeWheels(0.9);
        intakeServoLeft.setPosition(.6);
        intakeServoRight.setPosition(.9);
    }

    public void clampStone() { //servos to close position
        intakeServoLeft.setPosition(.62);
        intakeServoRight.setPosition(.92);
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