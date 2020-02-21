package org.firstinspires.ftc.teamcode.Trash;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Exponential_Methods;

@TeleOp(name="Monkey Move")
public class MonkeyMove extends Exponential_Methods {
    public void move(double inchesX, double inchesY, double tolerance){
        while((Math.abs(convertInchToEncoderOdom(inchesX)-odoWheelSideways.getCurrentPosition())>convertInchToEncoderOdom(tolerance)
                ||Math.abs(convertInchToEncoderOdom(inchesY)-odoWheelForwards.getCurrentPosition())>convertInchToEncoderOdom(tolerance))&&opModeIsActive()){
            double magnitude = Math.sqrt((convertInchToEncoderOdom(inchesX)-odoWheelSideways.getCurrentPosition())*(convertInchToEncoderOdom(inchesX)-odoWheelSideways.getCurrentPosition())
                    +(convertInchToEncoderOdom(inchesY)-odoWheelForwards.getCurrentPosition())*(convertInchToEncoderOdom(inchesY)-odoWheelForwards.getCurrentPosition()));
            double[] motorPowers = circle_to_taxicab((convertInchToEncoderOdom(inchesX)-odoWheelSideways.getCurrentPosition())/magnitude,(convertInchToEncoderOdom(inchesY)-odoWheelForwards.getCurrentPosition())/magnitude, 0 );

            double factor = 1;
            frontRight.setPower(factor*motorPowers[0]);
            backRight.setPower(factor*motorPowers[1]);
            backLeft.setPower(factor*motorPowers[2]);
            frontLeft.setPower(factor*motorPowers[3]);
        }
        setPowerDriveMotors(0);
    }
}
