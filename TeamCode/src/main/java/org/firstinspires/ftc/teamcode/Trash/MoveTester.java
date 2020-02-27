package org.firstinspires.ftc.teamcode.Trash;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Exponential_Methods;

@TeleOp(name = "MoveTester", group = "TeleOp")

public class MoveTester extends Exponential_Methods {

    double p = -0.00005;
    double i = -0.000017;
    double d = 0.00001;
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        double sideways = 36;
        double forwards = 0;
        telemetry.addData("It got up here", "hello");
        telemetry.update();
        waitForStart();
        while(opModeIsActive()) {
            if(gamepad1.left_stick_x!=0||gamepad1.left_stick_y!=0||gamepad1.right_stick_x!=0||gamepad1.right_stick_y!=0||gamepad2.right_stick_x!=0) {
                sideways += 1*(int)(gamepad1.left_stick_x);
                forwards -= 1*(int)(gamepad1.left_stick_y);
                p+=0.000001*(int)(gamepad1.right_stick_x);
                i-=0.000001*(int)(gamepad1.right_stick_y);
                d+=0.000001*(int)(gamepad2.right_stick_x);
                sleep(200);

            }
            telemetry.addData("forwards", forwards);
            telemetry.addData("sideways", sideways);
            telemetry.addData("p value", p);
            telemetry.addData("i value", i);
            telemetry.addData("d value", d);


            telemetry.update();
            if(gamepad1.a){
                move(sideways, forwards, p, i, d, 1);
            }
            telemetry.update();
        }
    }

    public void move(double inchesSideways, double inchesForward, double Kp, double Ki, double Kd, double inchesTolerance) {
        double pRot = .02;
        inchesSideways = -inchesSideways;
        odoWheelSideways.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        odoWheelForwards.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        odoWheelSideways.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        odoWheelForwards.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double toleranceEncoder = convertInchToEncoderOdom(inchesTolerance);
        double xTarget = convertInchToEncoderOdom(inchesSideways);
        double yTarget = convertInchToEncoderOdom(inchesForward);
        double minSpeed = 0;// 0.005; // Change later

        double disFront = yTarget; // Y displacement from the target in encoders
        double disSide = xTarget; // X displacement from the target in encoders
        double speedFront = 0; // Y speed in encoders per second
        double speedSide = 0; // X speed in encoders per second
        double areaFront = 0; // Y area in encoders * seconds
        double areaSide = 0; // X area in encoders * seconds

        double frontOdometryLastPosition = odoWheelForwards.getCurrentPosition();
        double sidewaysOdometryLastPosition = odoWheelSideways.getCurrentPosition();

        ElapsedTime interval = new ElapsedTime();
        ElapsedTime time = new ElapsedTime();
        while (opModeIsActive() && (Math.sqrt(Math.pow(disFront, 2) + Math.pow(disSide, 2))) > toleranceEncoder) {
            // Updates the area, displacement, and speed variables for the PID loop
            disFront = yTarget - odoWheelForwards.getCurrentPosition();
            disSide = xTarget - odoWheelSideways.getCurrentPosition();
            speedFront = (odoWheelForwards.getCurrentPosition() - frontOdometryLastPosition) / interval.seconds();
            speedSide = (odoWheelSideways.getCurrentPosition() - sidewaysOdometryLastPosition) / interval.seconds();
            areaFront += interval.seconds() * disFront;
            areaSide += interval.seconds() * disSide;

            double angle = getRotationInDimension('Z');
            // Sets the actual motor powers according to PID
            // Clips it so the motor power is not too low to avoid steady-state or goes too fast
            double maxPower = Math.min(.5+.5*time.seconds(), 1);
            frontLeft.setPower(motorClip(Kp * (disFront - disSide) + Ki * (areaFront - areaSide) + Kd * (speedFront - speedSide), minSpeed, maxPower));
            backRight.setPower(motorClip(Kp * (disFront - disSide) + Ki * (areaFront - areaSide) + Kd * (speedFront - speedSide), minSpeed, maxPower));
            frontRight.setPower(motorClip(Kp * (disFront + disSide) + Ki * (areaFront + areaSide) + Kd * (speedFront + speedSide), minSpeed, maxPower));
            backLeft.setPower(motorClip(Kp * (disFront + disSide) + Ki * (areaFront + areaSide) + Kd * (speedFront + speedSide), minSpeed, maxPower));

            telemetry.addData("Motor Front Left and Back Right", frontLeft.getPower());
            telemetry.addData("Motor Front Right and Back Left", frontRight.getPower());
            telemetry.update();

            frontOdometryLastPosition = odoWheelForwards.getCurrentPosition();
            sidewaysOdometryLastPosition = odoWheelSideways.getCurrentPosition();
            interval.reset();
        }
        setPowerDriveMotors(0);
    }
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
