package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class MethodsTesting extends LinearOpMode {
    //upleft, upright, backleft, backright
    private DcMotor[] driveMotors = new DcMotor[4];

    @Override
    public void runOpMode() throws InterruptedException {
        while(opModeIsActive()){
            driveMotors[0] = hardwareMap.dcMotor.get("upleft");
            driveMotors[1] = hardwareMap.dcMotor.get("upright");
            driveMotors[2] = hardwareMap.dcMotor.get("backleft");
            driveMotors[3] = hardwareMap.dcMotor.get("backright");

            driveMotors[0].setDirection(DcMotorSimple.Direction.REVERSE);
            driveMotors[2].setDirection(DcMotorSimple.Direction.REVERSE);


            move(5,5,1);
        }
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

    public void turnRelative(double targetChange, double speed) {

    }

    public void turnAbsolute(double targetAngle, double speed){

    }

}

