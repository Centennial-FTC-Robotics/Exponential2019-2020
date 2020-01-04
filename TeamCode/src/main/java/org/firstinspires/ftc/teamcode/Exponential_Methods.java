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

    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";
    private static final int MIDDLE_SCREEN = 640;
    private static final int TOP_MIDDLE_SCREEN = 360;

    private static final double TILE_LENGTH = 22.75;
    private static final double ROBOT_LENGTH = 18;
    private static final double BLOCK_LENGTH = 8;
    private static final double FOUNDATION_WIDTH = 18.5;
    private static final double FOUNDATION_AWAY_FROM_WALL = 4;
    //limits
    public static final int slideUpMax = 1500;
    public static final int slideDownMax = 3250;
    public static final int slideUpMin = -500;
    public static final int slideDownMin = -350;

    public static final int slideMax = 2300;
    public static final int slideMin = -400;

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        initializeIMU();
        initVuforia();
        initTfod();
    }

    //-------------- Initialization --------------
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

    //-------------- Basic --------------

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

    //-------------- Status --------------

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


    //-------------- Movement --------------

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

    //distance in inches
    public void oldMove(double forward, double right, double power) {

        for (DcMotor motor : driveMotors) {
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        for (DcMotor motor : driveMotors) {
            motor.setPower(power);
        }

        waitForMotors();

        int forwardVal = convertInchToEncoder(forward);
        int rightVal = convertInchToEncoder(right);

        frontLeft.setTargetPosition(forwardVal + rightVal);
        frontRight.setTargetPosition(forwardVal - rightVal);
        backLeft.setTargetPosition(forwardVal - rightVal);
        backRight.setTargetPosition(forwardVal + rightVal);


        setPowerDriveMotors(0);
        //return motors to original runmode
        for (DcMotor motor : driveMotors) {
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void move(double inchesSideways, double inchesForward, double maxPower){
        double targetAngle = getRotationinDimension('Z');
        double currentAngle;
        int direction;
        double turnRate;
        double P = 0.015; //set later
        double maxSpeed = 0.7; //set later
        double minSpeed = 0.01; //set later
        double error;









        inchesForward = -inchesForward;
        inchesSideways = getTransformedDistance(inchesSideways);

        double p = 1.0/1200;
        double i;
        double d;
        double inchesTolerance = 1.2;
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

    public void turnRelative(double targetAngle) {
        turnAbsolute(AngleUnit.normalizeDegrees(getRotationinDimension('Z') + targetAngle));
    }

    public void turnAbsolute(double targetAngle) {
        double currentAngle;
        int direction;
        double turnRate;
        double P = 0.01; //set later
        double tolerance = 4; //set later
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

    public void setSlidePower(double power){
        telemetry.addData("power",power);
        // telemetry.update();
        slideUp.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideDown.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        int currentPos = (slideUp.getCurrentPosition() + slideDown.getCurrentPosition()) /2;

        if(power == 0){
            slideUp.setPower(0);
            slideDown.setPower(0);
        }

        if(currentPos <= slideMax && currentPos >= slideMin){
            slideUp.setPower(power);
            slideDown.setPower(power);
        }

        while(opModeIsActive() && slideUp.isBusy() || slideDown.isBusy()) {
            currentPos = (slideUp.getCurrentPosition() + slideDown.getCurrentPosition()) /2;
            if (currentPos > slideMax) {
                if (power > 0) {
                    slideUp.setPower(-power);
                    slideDown.setPower(-power);
                }
            } else if (currentPos < slideMin) {
                if (power < 0) {
                    slideUp.setPower(-power);
                    slideDown.setPower(-power);
                }
            } else {
                slideUp.setPower(0);
                slideDown.setPower(0);
            }
        }

        if(!opModeIsActive()){
            slideUp.setPower(0);
            slideDown.setPower(0);
        }
    }

    public void extendSlidesBy(int inches, double speed){


        int encoderVal = convertInchToEncoderSlides(inches);

        slideUp.setTargetPosition(slideUp.getCurrentPosition() + encoderVal);
        slideDown.setTargetPosition(slideDown.getCurrentPosition() + encoderVal);
        slideUp.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideDown.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        //setSlidePower(speed);

        while(opModeIsActive() && (slideUp.isBusy() || slideDown.isBusy())){}
        setSlidePower(0);
    }

    /*
    //Slides are PAINFUL
    public void extendSlidesEncoder(int upVal, int downVal, double speed) {
        slideUp.setMode(DcMotor.RunMode.RUN__POSITION);
        slideDown.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        slideUp.setTargetPosition(upVal);
        slideDown.setTargetPosition(downVal);

        while(opModeIsActive() && slideUp.getCurrentPosition() < slideUpMax
                && slideDown.getCurrentPosition() < slideDownMin &&
                slideUp.getCurrentPosition() > slideUpMin &&
                slideDown.getCurrentPosition() > slideDownMin &&
                (slideUp.isBusy() || slideDown.isBusy())){
            setSlidePower(speed);
        }
        setSlidePower(0);

        slideUp.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideDown.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    //position in inches
    //PROBABLY DOESNT WORK
    public void extendSlidesTo(int position, double speed) {
        int encoderVal = convertInchToEncoderSlides(position);
        extendSlidesEncoder(encoderVal, encoderVal, speed);
    }

    //distance in inches
    public void extendSlidesBy(int distance, double speed) {
        int encoderVal = convertInchToEncoderSlides(distance);
        extendSlidesEncoder(encoderVal + slideUp.getCurrentPosition(),
                encoderVal + slideDown.getCurrentPosition(), speed);
    }


    //Positive = extend, negative = retract
    public void setSlidePower(double power) {
        slideUp.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideDown.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while(opModeIsActive() && slideUp.getCurrentPosition() < slideUpMax
                && slideDown.getCurrentPosition() < slideDownMax &&
                slideUp.getCurrentPosition() > slideUpMin &&
                slideDown.getCurrentPosition() > slideDownMin){
            slideUp.setPower(power);
            slideDown.setPower(power);
            telemetry.addData("setslidepower", power);
        }
        slideUp.setPower(0);
        slideDown.setPower(0);
    }
     */

    //Negative = backwards, positive = forwards
    public void setIntakeWheels(double power) {
        intakeLeft.setPower(power);
        intakeRight.setPower(power);
    }

    public void releaseStone() {
        setIntakeWheels(0);
        setIntakeServosPosition(0.7);
    }

    public void intakeStone() {
        setIntakeWheels(0.7);
        setIntakeServosPosition(1);
    }

    public void clampStone() {
        setIntakeWheels(0);
        setIntakeServosPosition(1);
    }


    //hook for moving foundation, true = down, false = up
    public void toggleHook(boolean down) {
        if (down) {
            hookServoLeft.setPosition(1);
            hookServoRight.setPosition(1);
        } else {
            hookServoLeft.setPosition(0);
            hookServoRight.setPosition(0);
        }
    }

    //servos that clamp
    public void setIntakeServosPosition(double position) {
        intakeServoLeft.setPosition(position);
        intakeServoRight.setPosition(position);
    }

    //-------------- Computer Vision --------------

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
            intakeStone();
            while (!center && blocksMoved < 3) {  // should move 3 blocks at max, otherwise vision doesn't work, move on
                move(factor * Math.sqrt(2) * 4, -Math.sqrt(2) * 4, 0.2);
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                blocksMoved++;
                sleep(500);
                if (updatedRecognitions != null) {
                    for (Recognition recognition : updatedRecognitions) {
                        telemetry.addData(String.format("label (%d)", 0), recognition.getLabel());
                        if (recognition.getLabel().equals(LABEL_SECOND_ELEMENT)) {
                            float stonePos = (recognition.getRight() + recognition.getLeft()) / 2;

                            //float stonePos = (recognition.getTop() + recognition.getBottom()) / 2;
                            center = stonePos + 160 > TOP_MIDDLE_SCREEN && TOP_MIDDLE_SCREEN > stonePos - 160;
                        }
                    }
                }
            }
            setPowerDriveMotors(0);
            clampStone();
        }

        //move forward, grab block, move back
        move(0, 12, 0.3);
        sleep(500);
        move(0, -12, 0.3);

        //turns back
        turnRelative(factor * -45);

        return blocksMoved * 8;
    }

    //-------------- Autonomous Paths --------------

    public void cornerAuto(String color, boolean second) { // starts on second tile from the side
        int factor;
        if (color.equals("red"))
            factor = 1;
        else
            factor = -1;

        //coordinates are for red side, they represent the location of the bottom left point of the robot from our POV
        // no matter what direction the robot is facing. done to hopefully reduce confusion cause fuck trying to
        //figure out what was going on

        //ok, yuhwan

        // ROBOT DISTANCE AWAY FROM BLOCK
        // this variable determines how far away from the block we want the robot when using grabSkystone
        // using this variable, calculate the distance the robot must travel to get the middle of stone exactly
        // in robot's field of sight
        double observingDistance = 12;
        // these two variables are separate for code clarity
        // idk what I would name a variable that represents both of these values
        double observingDistanceX = observingDistance / Math.sqrt(2);
        double observingDistanceY = observingDistance / Math.sqrt(2);

        // (1 tile, 0)

        double forwardToGetStone = 2 * TILE_LENGTH - ROBOT_LENGTH - observingDistanceY;
        move(-factor * (TILE_LENGTH - observingDistanceX), 0, 0.5); //move to corner //(observing distance x, 0)
        move(0, forwardToGetStone, 0.5); //move forward towards stones //(obs. dist. x, forwardToGetStone)
        int inchesMoved = grabSkystone(color); //(x, 2 tiles - robot length - observing dist. y)

        move(0, -forwardToGetStone, 0.5); //move back (can be cut out) //(x + obs. dist. x, 0)

        double alignToFoundationEdge = TILE_LENGTH - ROBOT_LENGTH - FOUNDATION_AWAY_FROM_WALL;

        move(factor * (TILE_LENGTH * 5 - inchesMoved - observingDistanceX + alignToFoundationEdge) , 0, 0.5); //(move through alliance bridge // (5 tiles + alignToFoundationEdge, 0)

        extendSlidesBy(3, 0.5); //move slides up to be able to go close to foudndation
        //move(TILE_LENGTH * 2 - ROBOT_LENGTH, 0, 0.5); //move to foundation // (6 tiles, tile - robot length)
        move(0, TILE_LENGTH * 2 - ROBOT_LENGTH, 0.5); //move to foundation // (5 tiles + alignToFoundationEdge, 2 tiles - robot length)

        releaseStone(); //drop stone out

        //moving foundation

        //moving forwards & backwards so corner of robot doesn't hit foundation
        move(0, -6, .5); // (5 tiles + alignToFoundationEdge, 2 tiles - robot length - 6 inches)
        turnAbsolute(180); //turn around
        //tile length - robot length - found. away from wall: aligns robot to the very edge of the foundation
        move(0, -6, .5); // (5 tiles + alignToFoundationEdge, 2 tiles - robot length)
        toggleHook(true); //grab foundation

        //moving robot away from any edge to try to stop conflicts from foundation turning, magic number away: 8

        //move to wall // (6 tiles - robot length - foundation width - 8, 8)
        move(factor * (-FOUNDATION_AWAY_FROM_WALL + 8 + FOUNDATION_WIDTH), TILE_LENGTH * 2 - ROBOT_LENGTH - 8, 0.5);
        turnRelative(factor * 90);
        //moving foundation all the way to corner
        move(-8, -8, .5); // (6 tiles - robot length - foundation width, 0)
        toggleHook(false);

        double tempPosition = 6 * TILE_LENGTH - ROBOT_LENGTH - FOUNDATION_WIDTH;
        extendSlidesBy(-3, 0.5); //move slides back down

        if (!second) { //if don't want second block

            // robot currently facing sideways
            // middle of robot will hopefully be on tape this way
            move(0, tempPosition - (3 * TILE_LENGTH - ROBOT_LENGTH / 2), 0.5); //parks on tape // (3 tiles - half of robot length, 0)

        } else { // if want second block

            //to try to get the second block
            move(0, tempPosition - 3 * BLOCK_LENGTH - observingDistanceX, .5); //move to second set of blocks // (3 blocks + obs. dist. x, 0)
            turnAbsolute(0); //turn back forwards
            move(0, forwardToGetStone, 0.5); //move forward to block // (3 blocks + obs. dist. x, forwardToGetStone)
            inchesMoved = grabSkystone(color); //grabbed block // (3 blocks + x, robot length)
            move(0, -forwardToGetStone, .5); //move back // (3 blocks + x + obs. dist. x, 0)
            turnAbsolute(90 * factor); //turn towards foundation, then move forwards

            //move slides up to be able to move close to foundation to drop
            extendSlidesBy(3, .5);

            //moving to the edge of the foundation
            // (6 blocks - foundation  - robot length, 0)
            move(0, 6 * TILE_LENGTH - FOUNDATION_WIDTH - ROBOT_LENGTH - (BLOCK_LENGTH * 3 + inchesMoved + observingDistanceX), .5);

            releaseStone();

            //moving backwards towards tape
            move(0, -1 * (TILE_LENGTH * 6 - FOUNDATION_WIDTH - ROBOT_LENGTH - (3 * TILE_LENGTH - ROBOT_LENGTH / 2)), 0.5); // (3 blocks - half of robot length, 0);

            //move slides back down (not necessary but good to have)
            extendSlidesBy(-3, 0.5);
        }
    }

    public void bridgeAuto(String color) { // we shouldn't get second block for bridge autonomous, will not move foundation
        int factor;
        if (color.equals("red"))
            factor = 1;
        else
            factor = -1;

        double observingDistance = 12;
        double observingDistanceX = observingDistance / Math.sqrt(2);
        double observingDistanceY = observingDistance / Math.sqrt(2);

        double forwardToGetStone = 2 * TILE_LENGTH - ROBOT_LENGTH - observingDistanceY;

        // (2 tiles, 0)
        move(-factor * (2 * TILE_LENGTH - 3 * BLOCK_LENGTH - observingDistanceX), 0, .5); // (3 blocks + obs. dist. x, 0)
        move(0, forwardToGetStone, .5); // (3 blocks + obs. dist. x, forwardToGetStone)
        int inchesMoved = grabSkystone(color); // (3 blocks + obs. dist. x + x, forwardToGetStone)

        move(factor * (4 * TILE_LENGTH + TILE_LENGTH / 2 - (3 * BLOCK_LENGTH + observingDistanceX + inchesMoved)), 0, .5); // (4.5 tiles, forwardToGetStone)

        extendSlidesBy(3, 0.5);
        move(0, observingDistanceY, .5); // (4.5 tiles, 2 tiles - robot length)
        releaseStone();
        //moves robot to the middle of the second tile
        move(0, -1 * (TILE_LENGTH - ROBOT_LENGTH) / 2, .5); // (4.5 tiles, centered on second tile)
        extendSlidesBy(-3, 0.5);

        move(-factor * (4.5 * TILE_LENGTH - 3 * TILE_LENGTH + ROBOT_LENGTH / 2), 0, .5); // (3 tiles - half robot, centered on second tile)


    }


}