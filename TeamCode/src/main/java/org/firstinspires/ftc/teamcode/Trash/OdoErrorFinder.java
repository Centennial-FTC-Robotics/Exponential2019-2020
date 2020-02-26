package org.firstinspires.ftc.teamcode.Trash;

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
            odoWheelForwards.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            odoWheelSideways.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            turnRelative(90);
            sleep(1000);
            odoForwardsChange = odoWheelForwards.getCurrentPosition() - odoForwardsError*90;
            odoSidewaysChange = odoWheelSideways.getCurrentPosition() - odoSidewaysError*90;

            odoForwardsError+=odoForwardsChange/90;
            odoSidewaysError+=odoSidewaysChange/90;
            telemetry.addData("OdoForwardsError", odoForwardsError);
            telemetry.addData("OdoSidewaysError", odoSidewaysError);
            telemetry.addData("OdoForwardsChange", odoForwardsChange);
            telemetry.addData("OdoSidewaysChange", odoSidewaysChange);
            telemetry.update();
        } while(opModeIsActive());
    }
}