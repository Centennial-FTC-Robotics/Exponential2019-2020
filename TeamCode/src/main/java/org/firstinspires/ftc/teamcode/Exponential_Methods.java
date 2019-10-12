package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public abstract class Exponential_Methods extends Exponential_Hardware_Initializations {

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
    }

    public void setPowerDriveMotors(double frontRight, double backRight, double backLeft, double frontLeft) {
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
    public void setRotateSpeed(double counterClockwise){
        setPowerDriveMotors(counterClockwise, counterClockwise, -counterClockwise, -counterClockwise);
    }

    public int convertInchToEncoder(double inches) {
        //get the right value ok
        int encoderValue = (int) Math.round(50 * inches);
        return encoderValue;
    }

    //distance in inches
    public void move(double forward, double right, double power) {

        int forwardVal = convertInchToEncoder(forward);
        int rightVal = convertInchToEncoder(right);

        for(DcMotor motor : driveMotors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setPower(power);
        }

        driveMotors[0].setTargetPosition(forwardVal - rightVal);
        driveMotors[1].setTargetPosition(forwardVal + rightVal);
        driveMotors[2].setTargetPosition(forwardVal + rightVal);
        driveMotors[3].setTargetPosition(forwardVal - rightVal);

        while (motorsBusy() && opModeIsActive()) {

        }
        setPowerDriveMotors(0);
        //return motors to original runmode
        for (DcMotor motor : driveMotors) {
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public boolean motorsBusy() {
        return driveMotors[0].isBusy() || driveMotors[1].isBusy() || driveMotors[2].isBusy() || driveMotors[3].isBusy();
    }

    public void turnRelative(double counterClockwiseAngle, double max, double min, double tolerance) {
        double constant = 1.0;

    }

    public void turnAbsolute(double counterClockwiseAngle, double max, double min, double tolerance) {

    }

    public void resetEncoders() {
        for(DcMotor motor: driveMotors){
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
}
