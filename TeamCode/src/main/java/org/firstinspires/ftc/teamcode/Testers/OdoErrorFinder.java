package org.firstinspires.ftc.teamcode.Testers;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Exponential_Methods;

@TeleOp(name="Odo Rotate Offset")
public class OdoErrorFinder extends Exponential_Methods {
    // tries to find the encoders per degree the sideways and forwards odometry wheels rotate
    public void runOpMode() throws InterruptedException{
        super.runOpMode();
        double odoSidewaysChange = 0; // encoders per degree of the sideways odometry wheel
        double odoForwardsChange = 0; // encoders per degree of the forwards odometry wheel
        double odoSidewaysError = 0;
        double odoForwardsError = 0;

        do{
            initializeIMU();
            odoWheelForwards.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            odoWheelSideways.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            turnRelative(90);
            sleep(1000);
            odoForwardsChange = odoWheelForwards.getCurrentPosition() - odoForwardsError*getRotationInDimension('Z');
            odoSidewaysChange = odoWheelSideways.getCurrentPosition() - odoSidewaysError*getRotationInDimension('Z');

            odoForwardsError+=odoForwardsChange/getRotationInDimension('Z')/2;
            odoSidewaysError+=odoSidewaysChange/getRotationInDimension('Z')/2;
            telemetry.addData("OdoForwardsError", odoForwardsError);
            telemetry.addData("OdoSidewaysError", odoSidewaysError);
            telemetry.addData("OdoForwardsChange", convertEncoderToInchOdom(odoForwardsChange));
            telemetry.addData("OdoSidewaysChange", convertEncoderToInchOdom(odoSidewaysChange));
            telemetry.update();
        } while(opModeIsActive());
    }
}