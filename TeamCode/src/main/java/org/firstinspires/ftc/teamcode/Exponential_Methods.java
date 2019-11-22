package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotor;
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
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import static org.firstinspires.ftc.robotcore.external.tfod.TfodRoverRuckus.TFOD_MODEL_ASSET;

public abstract class Exponential_Methods extends  Exponential_Hardware_Initializations {

    //tensor flow and vuforia stuff
    private static final String VUFORIA_KEY = "AQmuIUP/////AAAAGR6dNDzwEU07h7tcmZJ6YVoz5iaF8njoWsXQT5HnCiI/oFwiFmt4HHTLtLcEhHCU5ynokJgYSvbI32dfC2rOvqmw81MMzknAwxKxMitf8moiK62jdqxNGADODm/SUvu5a5XrAnzc7seCtD2/d5bAIv1ZuseHcK+oInFHZTi+3BvhbUyYNvnVb0tQEAv8oimzjiQW18dSUcEcB/d6QNGDvaDHpxuRCJXt8U3ShJfBWWQEex0Vp6rrb011z8KxU+dRMvGjaIy+P2p5GbWXGJn/yJS9oxuwDn3zU6kcQoAwI7mUgAw5zBGxxM+P35DoDqiOja6ST6HzDszHxClBm2dvTRP7C4DEj0gPkhX3LtBgdolt";
    public VuforiaLocalizer vuforia; //Vuforia localization engine
    public TFObjectDetector tfod; //Tensor Flow Object Detection engine
    private int cameraMonitorViewId;

    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";


    //limits
    public static final int slidesMax = 5; //set later
    public static final int slidesMin = 0; //set later

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
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

    public int convertInchToEncoder(double inches){
        double conversion = 31.01439480317557;
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

    public void waitForMotors(){
        while(motorsBusy() && opModeIsActive()){
        }
    }

    public void resetDriveMotorEncoders(){
        for(DcMotor motor : driveMotors)
            resetMotorEncoder(motor);
    }

    public void resetOrientation(){
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

    public boolean hasBlock(){
        if(sensorDistance.getDistance(DistanceUnit.INCH) < 1){
            return true;
        }
        return false;
    }


    //-------------- Movement --------------

    public void setPowerDriveMotors (double frontRight, double backRight, double backLeft, double frontLeft){
        super.frontRight.setPower(frontRight);
        super.frontLeft.setPower(frontLeft);
        super.backLeft.setPower(backLeft);
        super.backRight.setPower(backRight);
    }

    public void setPowerDriveMotors(float leftSpeed, float rightSpeed){
        frontLeft.setPower(Range.clip(leftSpeed, -1, 1));
        backLeft.setPower(Range.clip(leftSpeed, -1, 1));
        frontRight.setPower(Range.clip(rightSpeed, -1, 1));
        backRight.setPower(Range.clip(rightSpeed, -1, 1));
    }

    public void setPowerDriveMotors(double power) {
        for (DcMotor motor: driveMotors) {
            motor.setPower(power);
        }
    }

    //distance in inches
    public void oldMove(double forward, double right, double power){

        for(DcMotor motor : driveMotors) {
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        for(DcMotor motor : driveMotors) {
            motor.setPower(power);
        }

        waitForMotors();

        int forwardVal = convertInchToEncoder(forward);
        int rightVal = convertInchToEncoder(right);

        frontLeft.setTargetPosition(forwardVal + rightVal);
        frontRight.setTargetPosition(forwardVal - rightVal);
        backLeft.setTargetPosition(forwardVal - rightVal );
        backRight.setTargetPosition(forwardVal + rightVal);


        setPowerDriveMotors(0);
        //return motors to original runmode
        for(DcMotor motor : driveMotors){
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void move(double inchesForward, double inchesSideways, double maxPower){
        inchesSideways = -inchesSideways;
        double p = 1.0/800;
        double i;
        double d;
        double inchesTolerance = 0.5;
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

        while (opModeIsActive()&&(Math.abs(frontLeft_displacement)>tolerance||Math.abs(frontRight_displacement)>tolerance||Math.abs(backLeft_displacement)>tolerance||Math.abs(backRight_displacement)>tolerance)){
            frontLeft.setPower(Range.clip(p*frontLeft_displacement, min_negative, max_positive));
            frontRight.setPower(Range.clip(p*frontRight_displacement, min_negative, max_positive));
            backLeft.setPower(Range.clip(p*backLeft_displacement, min_negative, max_positive));
            backRight.setPower(Range.clip(p*backRight_displacement, min_negative, max_positive));

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
    }

    public void turnRelative(double targetAngle) {
        telemetry.addData("starting angle", getRotationinDimension(('z')));
        telemetry.update();
        turnAbsolute(0);
        turnAbsolute(AngleUnit.normalizeDegrees(getRotationinDimension('Z') + targetAngle));
    }

    public void turnAbsolute(double targetAngle){
        double currentAngle;
        int direction;
        double turnRate;
        double P = 0.001; //set later
        double tolerance = 4; //set later
        double maxSpeed = 0.4; //set later
        double minSpeed = 0.01; //set later
        double error;

        do{
            currentAngle = getRotationinDimension('Z');
            error = getAngleDist(targetAngle, currentAngle);
            direction = getAngleDir(targetAngle, currentAngle);
            turnRate = Range.clip(P * error, minSpeed, maxSpeed);
            telemetry.addData("error",error);
            telemetry.addData("turnRate", turnRate);
            telemetry.update();
            setPowerDriveMotors((float) (turnRate * direction), -(float) (turnRate * direction));
        }
        while(opModeIsActive() && error > tolerance);
        setPowerDriveMotors(0);
    }

    //position in inches
    //probably need to add negative signs and reverse stuff later when we actually have slides
    //add a minimum limit if necessary
    public void extendSlidesTo(int position, float speed){
        slideUp.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideDown.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        int encoderVal = position; //CONVERSION?

        slideUp.setTargetPosition(encoderVal);
        slideDown.setTargetPosition(encoderVal);

        while(opModeIsActive() && slideUp.getCurrentPosition() < slidesMax
                && slideDown.getCurrentPosition() < slidesMax &&
                slideUp.getCurrentPosition() > slidesMin &&
                slideDown.getCurrentPosition() > slidesMin &&
                (slideUp.isBusy() || slideDown.isBusy())){}
        setSlidePower(0);
    }

    //Positive = extend, negative = retract
    public void setSlidePower(double power){
        slideUp.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideDown.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        while(opModeIsActive() && slideUp.getCurrentPosition() < slidesMax
                && slideDown.getCurrentPosition() < slidesMax &&
                slideUp.getCurrentPosition() > slidesMin &&
                slideDown.getCurrentPosition() > slidesMin){
            slideUp.setPower(power);
            slideDown.setPower(power);
        }
    }

    //Negative = backwards, positive = forwards
    public void setIntakeWheels(double power){
        intakeLeft.setPower(power);
        intakeRight.setPower(power);
    }

    //hook for moving foundation, true = down, false = up
    public void toggleHook(boolean down){
        if(down)
            hookServo.setPosition(1);
        else
            hookServo.setPosition(0);
    }

    //servos that clamp
    public void setIntakeServosPosition(double position){
        intakeServoLeft.setPosition(position);
        intakeServoRight.setPosition(position);
    }

    //-------------- Computer Vision --------------

    public void findSkyStone(){

    }



}