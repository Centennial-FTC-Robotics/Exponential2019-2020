package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

public abstract class Exponential_Methods extends  Exponential_Hardware_Initializations {

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

    public void resetMotorEncoder(DcMotor motor) {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    //-------------- Status --------------

    public boolean motorsBusy() {
        return driveMotors[0].isBusy() || driveMotors[1].isBusy() || driveMotors[2].isBusy() || driveMotors[3].isBusy();
    }

    public void waitForMotors(){
        while(motorsBusy() && opModeIsActive()){
        }
    }

    public void resetDriveMotorsEncoders(){
        for(DcMotor motor : driveMotors)
            resetMotorEncoder(motor);
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

        frontLeft.setTargetPosition(forwardVal + rightVal);
        frontRight.setTargetPosition(forwardVal - rightVal);
        backLeft.setTargetPosition(forwardVal - rightVal );
        backRight.setTargetPosition(forwardVal + rightVal);

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

    public void turnRelative(int inchesForward, int inchesSideways, double p, double i, double d, double max, double min, double inchesTolerance){

    }

    public void move(double inchesForward, double inchesSideways, double p, double i, double d, double max, double min, double inchesTolerance){
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

        while (Math.abs(frontLeft_displacement)>tolerance&&Math.abs(frontRight_displacement)>tolerance&&Math.abs(backLeft_displacement)>tolerance&&Math.abs(backRight_displacement)>tolerance&&){
        }
        setPowerDriveMotors(0);
    }

    public void resetDriveMotorEncoders(){
        for (int i=0; i<driveMotors.length; i++) {
            driveMotors[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            driveMotors[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
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
    //PUT A LIMIT IN
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
