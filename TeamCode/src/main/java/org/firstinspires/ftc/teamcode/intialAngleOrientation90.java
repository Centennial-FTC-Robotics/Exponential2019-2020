package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Exponential_Methods;


@TeleOp(name = "Starting Angle 90")
public class intialAngleOrientation90 extends Exponential_Methods {
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

        decay.startingAngle = 90;
        while(opModeIsActive()){
            decay.updateRobot();
            telemetry.addData("X coord", convertEncoderToInchOdom(decay.currentX));
            telemetry.addData("Y coord", convertEncoderToInchOdom(decay.currentY));
            telemetry.update();
        }
    }
}
