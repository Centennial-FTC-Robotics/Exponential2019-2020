package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public abstract class Exponential_Methods extends  Exponential_Hardware_Initializations {

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
    }

    public void setPowerWheelMotors (double frontRight, double backRight, double backLeft, double frontLeft){
        super.frontRight.setPower(frontRight);
        super.frontLeft.setPower(frontLeft);
        super.backLeft.setPower(backLeft);
        super.backRight.setPower(backRight);
    }

    public void setRotateSpeed(double counterClockwise){
        setPowerWheelMotors(counterClockwise, counterClockwise, -counterClockwise, -counterClockwise);
    }

    public int convertInchToEncoder(double inches){
        //get the right value ok
        int encoderValue = (int) Math.round(50 * inches);
        return encoderValue;
    }

    //distance in inches
    public void move(double forward, double right, double power){

        int forwardVal = convertInchToEncoder(forward);
        int rightVal = convertInchToEncoder(right);

        for(DcMotor motor : driveMotors){
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setPower(power);
        }

        driveMotors[0].setTargetPosition(forwardVal - rightVal);
        driveMotors[1].setTargetPosition(forwardVal + rightVal);
        driveMotors[2].setTargetPosition(forwardVal + rightVal );
        driveMotors[3].setTargetPosition(forwardVal - rightVal);


        //return motors to original runmode
        for(DcMotor motor : driveMotors){
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void turnRelative(double counterClockwise) {

    }

    public void turnAbsolute(double counterClockwise){

    }
    
}
