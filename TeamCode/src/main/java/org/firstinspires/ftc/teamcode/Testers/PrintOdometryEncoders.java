package org.firstinspires.ftc.teamcode.Testers;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Exponential_Methods;

@TeleOp(name = "Print Odometry Encoders")
public class PrintOdometryEncoders extends Exponential_Methods {
    public void runOpMode() throws InterruptedException{
        super.runOpMode();
        frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        frontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        backLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        backRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        odoWheelForwards.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        odoWheelSideways.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        odoWheelForwards.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        odoWheelSideways.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        while(opModeIsActive()){
            telemetry.addData("forward Odometry Wheel", odoWheelForwards.getCurrentPosition());
            telemetry.addData("sideways Odometry Wheel", odoWheelSideways.getCurrentPosition());
            telemetry.update();
        }
    }
}