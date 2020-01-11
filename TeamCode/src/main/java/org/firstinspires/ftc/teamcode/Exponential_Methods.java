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

    public static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    public static final String LABEL_FIRST_ELEMENT = "Stone";
    public static final String LABEL_SECOND_ELEMENT = "Skystone";
    public static final int MIDDLE_SCREEN = 640;
    public static final int TOP_MIDDLE_SCREEN = 360;

    public static final double TILE_LENGTH = 22.75;
    public static final double ROBOT_LENGTH = 18;
    public static final double BLOCK_LENGTH = 8;
    public static final double FOUNDATION_WIDTH = 18.5;
    public static final double FOUNDATION_AWAY_FROM_WALL = 4;

    public static final double MIDDLE_OF_TILE = (TILE_LENGTH - ROBOT_LENGTH) / 2;

    public static final int slideMax = 2200;
    public static final int slideMin = -390;

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        initializeIMU();
        initVuforia();
        initTfod();
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
        double circumference = 3.613;
        double encoderToIn = 537.6;
        double conversion = encoderToIn / circumference;
        int encoderValue = (int) Math.round(conversion * inches);
        return encoderValue;
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

    public static final double DEFAULT_MOVE_TOLERANCE = .5; // SET DEFAULT TOLERANCE HERE
    
    public void move(double inchesSideways, double inchesForward, double maxPower){  
        // this one just inputs the default value (.5) for tolerance
        move(inchesSideways, inchesForward, maxPower, DEFAULT_MOVE_TOLERANCE);
    }

    public void moveAddTolerance(double inchesSideways, double inchesForward, double maxPower, double inchesToleranceAddition) {
        move(inchesSideways, inchesForward, maxPower, DEFAULT_MOVE_TOLERANCE + inchesToleranceAddition);
    }
    public void move(double inchesSideways, double inchesForward, double maxPower, double inchesTolerance){  // DON'T FUCK WITH THIS METHOD, i will find a better way to do this later
        double targetAngle = getRotationinDimension('Z');  // wow i found a good way to do this, good job yuhwan! ^^^
        double currentAngle;
        int direction;
        double turnRate;
        double P = 0.015; //set later
        double maxSpeed = 1; //set later
        double minSpeed = 0.01; //set later
        double error;

        inchesForward = -inchesForward;
        inchesSideways = getTransformedDistance(inchesSideways);

        double p = 1.0/1200;
        double i;
        double d;
        //double inchesTolerance = .5;
        double max_positive = maxPower;
        double min_negative = -maxPower;

        double encoderForward = convertInchToEncoder(inchesForward);
        double encoderSideways = convertInchToEncoder(inchesSideways);
        resetDriveMotorEncoders();
        double tolerance = convertInchToEncoder(inchesTolerance);

        double frontLeft_encoder = encoderForward-encoderSideways;
        double frontRight_encoder = encoderForward+encoderSideways;
        double backLeft_encoder = encoderForward+encoderSideways;
        double backRight_encoder = encoderForward-encoderSideways;

        double frontLeft_displacement = frontLeft_encoder-frontLeft.getCurrentPosition();
        double frontRight_displacement = frontRight_encoder-frontRight.getCurrentPosition();
        double backLeft_displacement = backLeft_encoder-backLeft.getCurrentPosition();
        double backRight_displacement = backRight_encoder-backRight.getCurrentPosition();
        ElapsedTime time = new ElapsedTime();
        while (opModeIsActive()&&(Math.abs(frontLeft_displacement)>tolerance||Math.abs(frontRight_displacement)>tolerance||Math.abs(backLeft_displacement)>tolerance||Math.abs(backRight_displacement)>tolerance)){
            currentAngle = getRotationinDimension('Z');

            error = getAngleDist(targetAngle, currentAngle);
            direction = getAngleDir(targetAngle, currentAngle);
            turnRate = Range.clip(P * error, minSpeed, maxSpeed);

            if(time.seconds()>2){
                frontLeft.setPower(Range.clip(p*frontLeft_displacement, min_negative, max_positive));
                frontRight.setPower(Range.clip(p*frontRight_displacement, min_negative, max_positive));
                backLeft.setPower(Range.clip(p*backLeft_displacement, min_negative, max_positive));
                backRight.setPower(Range.clip(p*backRight_displacement, min_negative, max_positive));
            } else {
                frontLeft.setPower(+direction * turnRate + Range.clip(p * frontLeft_displacement, min_negative, max_positive));
                frontRight.setPower(direction * turnRate + Range.clip(p * frontRight_displacement, min_negative, max_positive));
                backLeft.setPower(+direction * turnRate + Range.clip(p * backLeft_displacement, min_negative, max_positive));
                backRight.setPower(direction * turnRate + Range.clip(p * backRight_displacement, min_negative, max_positive));
            }

            frontLeft_displacement = frontLeft_encoder-frontLeft.getCurrentPosition();
            frontRight_displacement = frontRight_encoder-frontRight.getCurrentPosition();
            backLeft_displacement = backLeft_encoder-backLeft.getCurrentPosition();
            backRight_displacement = backRight_encoder-backRight.getCurrentPosition();
            telemetry.addData("frontLeft", frontLeft_displacement);
            telemetry.addData("backLeft", backLeft_displacement);
            telemetry.addData("frontRight", frontRight_displacement);
            telemetry.addData("backRight", backRight_displacement);
            telemetry.addData("tolerance", tolerance);
            telemetry.update();
        }
        setPowerDriveMotors(0);
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
        double P = 0.01; //set later
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

    public void setSlidePower(double power){
        slideUp.setPower(power);
        slideDown.setPower(power);
    }

    public void extendSlidesTo(int encoderPos, double power){
        if(encoderPos > slideMax)
            encoderPos = slideMax;
        else if(encoderPos < slideMin)
            encoderPos = slideMin;

        slideUp.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideDown.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideUp.setTargetPosition(encoderPos);
        slideDown.setTargetPosition(encoderPos);
        slideUp.setPower(power);
        slideDown.setPower(power);
    }


    public void extendSlidesBy(double inches, double speed){

        int position = (slideUp.getCurrentPosition() + slideDown.getCurrentPosition()) /2;
        int encoderVal = convertInchToEncoderSlides(inches);

        int val = position + encoderVal;

        if(position > slideMax)
            val = slideMax;
        else if(position < slideMin)
            val = slideMin;

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
        if (down) {
            hookServoLeft.setPosition(.75);
            hookServoRight.setPosition(.5);
        } else {
            hookServoLeft.setPosition(0.1);
            hookServoRight.setPosition(0);
        }
        sleep(500);
    }

    //servos that clamp
    public void setIntakeServosPosition(double position) {
        intakeServoLeft.setPosition(position);
        intakeServoRight.setPosition(position);
    }

    //-------------- AUTO AID METHODS  -------------- (organization)

    public void releaseStone() {
        setIntakeWheels(0);
        setIntakeServosPosition(0.7);
    }

    public void intakeStone() {
        setIntakeWheels(0.8);
        setIntakeServosPosition(1);
    }
    public void clampStone() {
        setIntakeServosPosition(1);
    }
    public void stopIntakeWheels() {  // COMBINE THESE INTO A NEW METHOD MAYBE
        setIntakeWheels(0);
    }
    /*public void clampStone() {
        setIntakeWheels(0);
        setIntakeServosPosition(1);
    }*/

    public void bringSlidesDown(){
        extendSlidesBy(4,0.5);
        sleep(1000);
        setIntakeServosPosition(0.7); // changed to release stone servo position so camera can see block
        extendSlidesTo(slideMin, 0.5);
    }

    public int grabSkystone(String color) {
        int factor;
        if (color.equals("red"))
            factor = 1;
        else
            factor = -1;
        turnRelative(factor * 45);

        boolean center = false;
        tfod.activate();

        int blocksMoved = 0;
        if (opModeIsActive()) {
            while (!center && blocksMoved < 3) {  // should move 3 blocks at max, otherwise vision doesn't work, move on
                moveAddTolerance(factor * Math.sqrt(2) * 4, -Math.sqrt(2) * 4, .5, .2);
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                blocksMoved++;
                sleep(500);
                if (updatedRecognitions != null) {
                    for (Recognition recognition : updatedRecognitions) {
                        telemetry.addData(String.format("label (%d)", 0), recognition.getLabel());
                        if (recognition.getLabel().equals(LABEL_SECOND_ELEMENT)) {
                            float stonePos = (recognition.getRight() + recognition.getLeft()) / 2;  // vertical phone im pretty sure

                            //float stonePos = (recognition.getTop() + recognition.getBottom()) / 2;
                            center = stonePos + 160 > TOP_MIDDLE_SCREEN && TOP_MIDDLE_SCREEN > stonePos - 160;
                        }
                    }
                }
            }
            // setPowerDriveMotors(0);
        }

        //move forward, grab block, move back
        intakeStone();

        move(0, 14, 0.3);
        clampStone();
        sleep(250);
        stopIntakeWheels();
        sleep(250);
        move(0, -14, 0.3);

        //turns back
        turnRelative(factor * -45);

        return blocksMoved * 8;
    }

    //-------------- AUTONOMOUS PATHS -------------- (organization)

    public void cornerAuto(String color, boolean second, boolean secondTilePath) { // starts on second tile from the side
        int factor;
        if (color.equals("red"))
            factor = 1;
        else
            factor = -1;

        //releaseStone();
        //setSlidesMinimum();
        //TODO AHHHHHHHHHHHHHHH UNCOMMENT THIS LATER
        /*bringSlidesDown();*/
        //coordinates are for red side, they represent the location of the bottom left point of the robot from our POV
        // no matter what direction the robot is facing. done to hopefully reduce confusion cause fuck trying to
        //figure out what was going on

        // ROBOT DISTANCE AWAY FROM BLOCK
        // this variable determines how far away from the block we want the robot when using grabSkystone
        // using this variable, calculate the distance the robot must travel to get the middle of stone exactly
        // in robot's field of sight
        double observingDistance = 10;
        // these two variables are separate for code clarity
        // idk what I would name a variable that represents both of these values
        double observingDistanceX = observingDistance / Math.sqrt(2);
        double observingDistanceY = observingDistance / Math.sqrt(2);

        double secondTile = TILE_LENGTH;
        double offsetForFoundation = 3; // TO BE CHANGED IF NEED BE
        // (1 tile, 0)

        double forwardToGetStone = 2 * TILE_LENGTH - ROBOT_LENGTH - observingDistanceY;
        moveAddTolerance(-factor * (TILE_LENGTH - observingDistanceX), 0, 0.5, .2); //move to corner //(observing distance x, 0)
        moveAddTolerance(0, forwardToGetStone, 0.5, .1); //move forward towards stones //(obs. dist. x, forwardToGetStone)
        int inchesMoved = grabSkystone(color); //(obs.dist.x + x, forwardsToGetStone)
        /*
        move(0, -forwardToGetStone, 0.5); //move back (can be cut out) //(x + obs. dist. x, 0)
        */
        int dontRunIntoWall = 1;
        if (!secondTilePath) { //MOVING BACK AFTER GETTING SKYSTONE
            moveAddTolerance(0, -forwardToGetStone + MIDDLE_OF_TILE + dontRunIntoWall, .5, .2);  // (x + obs. dist. x, middle of tile)
        } else {
            moveAddTolerance(0, -forwardToGetStone + MIDDLE_OF_TILE + dontRunIntoWall + secondTile, .5, .2);  // (x + obs. dist. x, middle of tile)
        }
        double alignToFoundationEdge = TILE_LENGTH - ROBOT_LENGTH - FOUNDATION_AWAY_FROM_WALL;
        turnAbsolute(factor * -90);
        // turnRelative(-90 * factor);
        move(0 , TILE_LENGTH * 5 - inchesMoved - observingDistanceX + alignToFoundationEdge, 0.5); //(move through alliance bridge // (5 tiles + alignToFoundationEdge, middle of tile)
        turnAbsolute(0);
        // turnRelative(90 * factor);

        extendSlidesBy(6, 0.5); //move slides up to be able to go close to foudndation

        //move(TILE_LENGTH * 2 - ROBOT_LENGTH, 0, 0.5); //move to foundation // (6 tiles, tile - robot length)
        // TODO: change 2 if need be
        //move(0, TILE_LENGTH * 2/* - ROBOT_LENGTH TODO see if this stays*//* - forwardToGetStone*/ - 2 - MIDDLE_OF_TILE, 0.5); //move to foundation // (5 tiles + alignToFoundationEdge, 2 tiles - robot length - 2)
        if (!secondTilePath) {
            move(0, TILE_LENGTH * 2 - ROBOT_LENGTH - MIDDLE_OF_TILE + offsetForFoundation, .5); // (5 tiles + alignToFoundationEdge, 2 tiles - robot length)
        } else {
            move(0, TILE_LENGTH * 2 - ROBOT_LENGTH - MIDDLE_OF_TILE + offsetForFoundation - secondTile, .5); // (5 tiles + alignToFoundationEdge, 2 tiles - robot length)
        }
        releaseStone(); //drop stone out

        //moving foundation

        //moving forwards & backwards so corner of robot doesn't hit foundation
        move(0, -6, .5); // (5 tiles + alignToFoundationEdge, 2 tiles - robot length - 6 inches)
        turnAbsolute(180); //turn around
        //tile length - robot length - found. away from wall: aligns robot to the very edge of the foundation
        moveAddTolerance(0, -6 - offsetForFoundation, .5, .2); // (5 tiles + alignToFoundationEdge, 2 tiles - robot length)
        toggleHook(true); //grab foundation

        //moving robot away from any edge to try to stop conflicts from foundation turning, magic number away: 8

        //move to wall // (6 tiles - robot length - foundation width - 8, 8)
        //TODO: 19 is a sketchy ass value
        moveAddTolerance(factor * (-FOUNDATION_AWAY_FROM_WALL + 8 + FOUNDATION_WIDTH), 0, .5, .2);
        moveAddTolerance(0, 19 + TILE_LENGTH * 2 - ROBOT_LENGTH - 8 - offsetForFoundation * 2, .5, .2);
        //moveAddTolerance(factor * (-FOUNDATION_AWAY_FROM_WALL + 8 + FOUNDATION_WIDTH), 19 + TILE_LENGTH * 2 - ROBOT_LENGTH - 8 - offsetForFoundation, 0.5, .2);

        turnAbsolute(180 - 90 * factor);
        // turnRelative(factor * -90);
        //moving foundation all the way to corner
        moveAddTolerance(factor * -8, 0, .5, .2);
        moveAddTolerance(0, -8, .5, .2);
        //moveAddTolerance( factor * -8, -8, .5, .2); // (6 tiles - robot length - foundation width, 0)
        toggleHook(false);
        if (!secondTilePath) {
            move(factor * MIDDLE_OF_TILE, 0, .5); //6 tiles - robot length - foundation width, middle of tile)
        } else {
            move(factor * (MIDDLE_OF_TILE + secondTile), 0, .5); //6 tiles - robot length - foundation width, middle of tile)
        }
        double tempPosition = 6 * TILE_LENGTH - ROBOT_LENGTH - FOUNDATION_WIDTH;
        extendSlidesBy(-6, 0.5); //move slides back down

        if (!second) { //if don't want second block

            // robot currently facing sideways
            // middle of robot will hopefully be on tape this way
            move(0, tempPosition - (3 * TILE_LENGTH - ROBOT_LENGTH / 2), 0.5); //parks on tape // (3 tiles - half of robot length, middle of tile)

        } else { // if want second block

            //to try to get the second block
            move(0, tempPosition - 3 * BLOCK_LENGTH - observingDistanceX, .5); //move to second set of blocks // (3 blocks + obs. dist. x, middle of tile)
            turnAbsolute(0); //turn back forwards
            if (!secondTilePath) {
                move(0, forwardToGetStone - MIDDLE_OF_TILE, 0.5); //move forward to block // (3 blocks + obs. dist. x, forwardToGetStone)
            } else {
                move(0, forwardToGetStone - MIDDLE_OF_TILE - secondTile, 0.5); //move forward to block // (3 blocks + obs. dist. x, forwardToGetStone)

            }
            inchesMoved = grabSkystone(color); //grabbed block // (3 blocks + x, robot length)
            if (!secondTilePath) {
                move(0, -forwardToGetStone + MIDDLE_OF_TILE, .5); //move back // (3 blocks + x + obs. dist. x, middle of tile)
            } else {
                move(0, -forwardToGetStone + MIDDLE_OF_TILE + secondTile, .5); //move back // (3 blocks + x + obs. dist. x, middle of tile)
            }
            turnAbsolute(-90 * factor); //turn towards foundation, then move forwards

            //move slides up to be able to move close to foundation to drop
            extendSlidesBy(6, .5);

            //moving to the edge of the foundation
            // (6 blocks - foundation  - robot length, middle of tile)
            move(0, 6 * TILE_LENGTH - FOUNDATION_WIDTH - ROBOT_LENGTH - (BLOCK_LENGTH * 3 + inchesMoved + observingDistanceX), .5);

            releaseStone();

            extendSlidesBy(-6, 0.5);

            //moving backwards towards tape
            move(0, -1 * (TILE_LENGTH * 6 - FOUNDATION_WIDTH - ROBOT_LENGTH - (3 * TILE_LENGTH - ROBOT_LENGTH / 2)), 0.5); // (3 blocks - half of robot length, tile length);

        }
    }

    public void bridgeAuto(String color) { // we shouldn't get second block for bridge autonomous, will not move foundation
        int factor;
        if (color.equals("red"))
            factor = 1;
        else
            factor = -1;

        bringSlidesDown();

        double observingDistance = 10;
        double observingDistanceX = observingDistance / Math.sqrt(2);
        double observingDistanceY = observingDistance / Math.sqrt(2);

        double forwardToGetStone = 2 * TILE_LENGTH - ROBOT_LENGTH - observingDistanceY;

        // (2 tiles, 0)
        move(-factor * (2 * TILE_LENGTH - 3 * BLOCK_LENGTH - observingDistanceX), 0, .5); // (3 blocks + obs. dist. x, 0)
        move(0, forwardToGetStone, .5); // (3 blocks + obs. dist. x, forwardToGetStone)
        int inchesMoved = grabSkystone(color); // (3 blocks + obs. dist. x + x, forwardToGetStone)

        turnAbsolute(factor * -90);
        move(0, 4 * TILE_LENGTH + TILE_LENGTH / 2 - (3 * BLOCK_LENGTH + observingDistanceX + inchesMoved), .5); // (4.5 tiles, forwardToGetStone)

        extendSlidesBy(6, 0.5);
        turnAbsolute(0);
        move(0, observingDistanceY, .5); // (4.5 tiles, 2 tiles - robot length)
        releaseStone();
        //moves robot to the middle of the second tile
        move(0, -1 * MIDDLE_OF_TILE, .5); // (4.5 tiles, centered on second tile)
        extendSlidesBy(-6, 0.5);

        turnAbsolute(factor * 90);
        move(0, 4.5 * TILE_LENGTH - 3 * TILE_LENGTH + ROBOT_LENGTH / 2, .5); // (3 tiles - half robot, centered on second tile)

    }



}
