package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotor;

public abstract class Exponential_Methods extends  Exponential_Hardware_Initializations {

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
    }

    //-------------- Basic --------------

    public int convertInchToEncoder(double inches){
        //get the right value ok
        int encoderValue = (int) Math.round(50 * inches);
        return encoderValue;
    }

    public boolean motorsBusy() {
        return driveMotors[0].isBusy() || driveMotors[1].isBusy() || driveMotors[2].isBusy() || driveMotors[3].isBusy();
    }

    public void waitForMotors(){
        while(motorsBusy() && opModeIsActive()){
        }
    }

    public void resetOrientation(){

    }

    //-------------- Movement --------------

    public void setPowerDriveMotors (double frontRight, double backRight, double backLeft, double frontLeft){
        super.frontRight.setPower(frontRight);
        super.frontLeft.setPower(frontLeft);
        super.backLeft.setPower(backLeft);
        super.backRight.setPower(backRight);
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

        for(DcMotor motor : driveMotors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setPower(power);
        }

        driveMotors[0].setTargetPosition(forwardVal - rightVal);
        driveMotors[1].setTargetPosition(forwardVal + rightVal);
        driveMotors[2].setTargetPosition(forwardVal + rightVal );
        driveMotors[3].setTargetPosition(forwardVal - rightVal);

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

    public void turnRelative(double counterClockwiseAngle, double max, double min, double tolerance) {
        double constant = 1.0;

    }

    public void turnAbsolute(double targetAngle, double speed){
        double currentAngle;

    }

    //What units should position be?
    public void extendSlidesTo(int position, float speed){
        slideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //SET POSITION but idk if we want parameter to be inch or encoder values
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
