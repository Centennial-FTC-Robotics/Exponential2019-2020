package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "TeleOp", group = "TeleOp")

public class TeleOpDriver extends Exponential_Methods {
    private double[] circle_to_taxicab(double circle_x, double circle_y) {
        double[] answer = new double[2];
        if (circle_x == 0.0)
            answer[0] = 0.0;
        else
            answer[0] = circle_x / Math.abs(circle_x) * Math.sqrt(Math.pow(circle_x, 2) + Math.pow(circle_y, 2))
                    * (Math.abs(circle_x)) / (Math.abs(circle_x) + Math.abs(circle_y));
        if (circle_y == 0.0)
            answer[1] = 0.0;
        else
            answer[1] = circle_y / Math.abs(circle_y) * Math.sqrt(Math.pow(circle_x, 2) + Math.pow(circle_y, 2))
                    * (Math.abs(circle_y)) / (Math.abs(circle_x) + Math.abs(circle_y));
        return answer;
    }

    private void taxicab_method(double powerright, double powerup) {
        double taxiright = circle_to_taxicab(powerright, powerup)[0];
        double taxiup = circle_to_taxicab(powerright, powerup)[1];
        frontLeft.setPower((-taxiright + taxiup));
        backLeft.setPower((taxiright + taxiup));
        frontRight.setPower((taxiright + taxiup));
        backRight.setPower((-taxiright + taxiup));
    }

    private void brute_force_method(double powerright, double powerup) {
        double magnitude = Math.sqrt(powerright * powerright + powerup * powerup);
        if (magnitude > 1) {
            frontLeft.setPower((-powerright + powerup) / magnitude);
            backLeft.setPower((powerright + powerup) / magnitude);
            frontRight.setPower((powerright + powerup) / magnitude);
            backRight.setPower((-powerright + powerup) / magnitude);
        } else {
            frontLeft.setPower((-powerright + powerup));
            backLeft.setPower((powerright + powerup));
            frontRight.setPower((powerright + powerup));
            backRight.setPower((-powerright + powerup));
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        waitForStart();

        while (opModeIsActive()) {
            double powerright = gamepad1.right_stick_x;
            double powerup = gamepad1.right_stick_y;
            double rotate_counterclockwise = gamepad1.left_stick_x;

            double trigger_factor = 1.0 - gamepad1.left_trigger;
            if (powerright != 0.0 || powerup != 0.0) {

                // change this to adjust top speed
                double adjustable_move_factor = 1.0;

                // Comment either taxicab or brute_force out
                taxicab_method(adjustable_move_factor * trigger_factor * powerright, adjustable_move_factor * trigger_factor * powerup);
                // brute_force_method(adjustable_move_factor * trigger_factor * powerright, adjustable_move_factor * trigger_factor * powerup);
            } else if (rotate_counterclockwise != 0.0) {
                // change this to adjust rotation speed
                double adjustable_rotate_factor = .5;

                frontLeft.setPower(adjustable_rotate_factor * trigger_factor * -rotate_counterclockwise);
                backLeft.setPower(adjustable_rotate_factor * trigger_factor * -rotate_counterclockwise);
                frontRight.setPower(adjustable_rotate_factor * trigger_factor * rotate_counterclockwise);
                backRight.setPower(adjustable_rotate_factor * trigger_factor * rotate_counterclockwise);
            } else {
                frontLeft.setPower(0);
                backLeft.setPower(0);
                frontRight.setPower(0);
                backRight.setPower(0);
            }

            //slides
            setSlidePower(Range.clip(gamepad2.left_stick_y,0,0.7)); //set max later

            //hook down
            if(gamepad1.x)
                toggleHook(true);

            //hook up
            if(gamepad1.b)
                toggleHook(false);

            //slide speed reduced to 1/2
            if(gamepad2.left_bumper)
                setSlidePower(slideUp.getPower() / 2);

            //slide speed reduced to 1/4
            if(gamepad2.right_bumper)
                setSlidePower(slideUp.getPower() / 4);

            //intake wheels
            double intakePower = 0.9; //set later
            setIntakeWheels(intakePower * gamepad2.left_trigger);
            setIntakeWheels(0.25 * intakePower * gamepad2.right_trigger);

            if(gamepad2.b){
                //intake servos clamp stone
                setIntakeServosPosition(1); //set later
            }

            if(gamepad2.x){
                //intake servos release stone
                setIntakeServosPosition(0.7); //set later BY MANUALLY TESTING HAHAHAHA
            }



        }
    }

}
