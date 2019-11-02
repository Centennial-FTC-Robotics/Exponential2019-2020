package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

public abstract class Exponential_Methods extends  Exponential_Hardware_Initializations {

    //tensor flow and vuforia stuff
    private static final String VUFORIA_KEY = "AQmuIUP/////AAAAGR6dNDzwEU07h7tcmZJ6YVoz5iaF8njoWsXQT5HnCiI/oFwiFmt4HHTLtLcEhHCU5ynokJgYSvbI32dfC2rOvqmw81MMzknAwxKxMitf8moiK62jdqxNGADODm/SUvu5a5XrAnzc7seCtD2/d5bAIv1ZuseHcK+oInFHZTi+3BvhbUyYNvnVb0tQEAv8oimzjiQW18dSUcEcB/d6QNGDvaDHpxuRCJXt8U3ShJfBWWQEex0Vp6rrb011z8KxU+dRMvGjaIy+P2p5GbWXGJn/yJS9oxuwDn3zU6kcQoAwI7mUgAw5zBGxxM+P35DoDqiOja6ST6HzDszHxClBm2dvTRP7C4DEj0gPkhX3LtBgdolt";
    private VuforiaLocalizer vuforia; //Vuforia localization engine
    private TFObjectDetector tfod; //Tensor Flow Object Detection engine
    private int cameraMonitorViewId;


    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
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

    //-------------- Status --------------

    public boolean motorsBusy() {
        return driveMotors[0].isBusy() || driveMotors[1].isBusy() || driveMotors[2].isBusy() || driveMotors[3].isBusy();
    }

    public void waitForMotors(){
        while(motorsBusy() && opModeIsActive()){
        }
    }

    public void resetOrientation(){

    }

    private void initVuforia() {
        //create parameter object and pass it to create Vuforia engine
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
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
    public void move(double forward, double right, double power){

        int forwardVal = convertInchToEncoder(forward);
        int rightVal = convertInchToEncoder(right);

        frontLeft.setTargetPosition(forwardVal - rightVal);
        frontRight.setTargetPosition(forwardVal + rightVal);
        backLeft.setTargetPosition(forwardVal + rightVal );
        backRight.setTargetPosition(forwardVal - rightVal);

        for(DcMotor motor : driveMotors) {
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        for(DcMotor motor : driveMotors) {
            motor.setPower(power);
        }

        waitForMotors();

        setPowerDriveMotors(0);
        //return motors to original runmode
        for(DcMotor motor : driveMotors){
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void setRotateSpeed(double counterClockwise){
        setPowerDriveMotors(counterClockwise, counterClockwise, -counterClockwise, -counterClockwise);
    }

    public void turnRelative(double targetAngle) {

    }

    public void turnAbsolute(double targetAngle){
        double currentAngle;
        int direction;
        double turnRate = 0;
        double P = 0.1; //set later
        double tolerance = 0.5; //set later
        double maxSpeed = 0.5; //set later
        double minSpeed = 0.02; //set later
        double error;

        do{
            currentAngle = 00000; //set later
            error = getAngleDist(targetAngle, currentAngle);
            direction = getAngleDir(targetAngle, currentAngle);
            turnRate = Range.clip(P * error, minSpeed, maxSpeed);
            setPowerDriveMotors((float) -(turnRate * direction), (float) (turnRate * direction));
        }
        while(opModeIsActive() && error > tolerance);
        setPowerDriveMotors(0);
    }

    //What units should position be?
    public void extendSlidesTo(int position, float speed){
        slideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //SET POSITION!! but idk if we want parameter to be inch or encoder values
        //so i left it blank for now
        //slideLeft.setTargetPosition(value??);
        //slideRight.setTargetPosition(value??);

        slideLeft.setPower(speed);
        slideRight.setPower(speed);

        while(opModeIsActive() && slideLeft.isBusy() || slideRight.isBusy()){}

        slideLeft.setPower(0);
        slideRight.setPower(0);
    }

    public void setSlidePower(float power){
        slideLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideLeft.setPower(power);
        slideRight.setPower(power);
    }

    //Negative = backwards, positive = forwards
    public void setIntakeWheels(double power){
        intakeLeft.setPower(power);
        intakeRight.setPower(power);
    }

    //Idk parameters yet; this is the hook for moving foundation
    public void toggleHook(){

    }

    //Idk parameters yet; this is the servos that allows the intake to clamp
    public void intakeToggle(){

    }

    //-------------- Computer Vision --------------

    public void findSkyStone(){

    }


    
}
