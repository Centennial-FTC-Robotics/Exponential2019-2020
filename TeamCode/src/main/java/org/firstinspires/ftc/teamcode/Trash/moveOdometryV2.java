package org.firstinspires.ftc.teamcode.Trash;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Exponential_Methods;

public class moveOdometryV2 extends Exponential_Methods {
    public void move(double inchesSideways, double inchesForward, double Kp, double Ki, double Kd, double inchesTolerance, double maxPower) {
        odoWheelSideways.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        odoWheelForwards.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        odoWheelSideways.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        odoWheelForwards.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double toleranceEncoder = convertInchToEncoderOdom(inchesTolerance);
        double xTarget = convertInchToEncoderOdom(inchesSideways);
        double yTarget = convertInchToEncoderOdom(inchesForward);
        double minSpeed = 0.03;

        double disFront = yTarget; // Y displacement from the target in encoders
        double disSide = xTarget; // X displacement from the target in encoders
        double speedFront = 0; // Y speed in encoders per second
        double speedSide = 0; // X speed in encoders per second
        double areaFront = 0; // Y area in encoders * seconds
        double areaSide = 0; // X area in encoders * seconds

        double frontOdometryLastPosition = 0;
        double sidewaysOdometryLastPosition = 0;

        ElapsedTime interval = new ElapsedTime();


        while (opModeIsActive() && (Math.sqrt(Math.pow(disFront, 2) + Math.pow(disSide, 2))) > toleranceEncoder) {
            // Updates the area, displacement, and speed variables for the PID loop
            disFront = yTarget - odoWheelForwards.getCurrentPosition();
            disSide = xTarget - odoWheelSideways.getCurrentPosition();
            speedFront = (odoWheelForwards.getCurrentPosition() - frontOdometryLastPosition) / interval.seconds();
            speedSide = (odoWheelSideways.getCurrentPosition() - sidewaysOdometryLastPosition) / interval.seconds();
            areaFront += interval.seconds() * disFront;
            areaSide += interval.seconds() * disSide;

            // Sets the actual motor powers according to PID
            // Clips it so the motor power is not too low to get into steady-state or too high
            frontLeft.setPower(motorClip(Kp * (disFront - disSide) + Ki * (areaFront - areaSide) + Kd * (speedFront - speedSide), minSpeed, maxPower));
            backRight.setPower(motorClip(Kp * (disFront - disSide) + Ki * (areaFront - areaSide) + Kd * (speedFront - speedSide), minSpeed, maxPower));
            frontRight.setPower(motorClip(Kp * (disFront + disSide) + Ki * (areaFront + areaSide) + Kd * (speedFront + speedSide), minSpeed, maxPower));
            backLeft.setPower(motorClip(Kp * (disFront + disSide) + Ki * (areaFront + areaSide) + Kd * (speedFront + speedSide), minSpeed, maxPower));

            frontOdometryLastPosition = odoWheelForwards.getCurrentPosition();
            sidewaysOdometryLastPosition = odoWheelSideways.getCurrentPosition();
            interval.reset();
        }
        setPowerDriveMotors(0);
    }


    // min Power and max Power have to be positive
    // keeps the motor from running too slowly or going too fast
    private double motorClip(double power, double minPower, double maxPower){
        if(power < 0){
            return Range.clip(power, -maxPower, -minPower);
        } else if (power > 0){
            return Range.clip(power, minPower, maxPower);
        } else {
            return 0;
        }
    }
}