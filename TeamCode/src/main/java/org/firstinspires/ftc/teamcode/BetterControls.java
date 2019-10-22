package org.firstinspires.ftc.teamcode;
// :3
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Controls", group = "TeleOp")
public class BetterControls extends LinearOpMode {
    private DcMotor upleft;
    private DcMotor backleft;
    private DcMotor upright;
    private DcMotor backright;

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
        upleft.setPower((-taxiright + taxiup));
        backleft.setPower((taxiright + taxiup));
        upright.setPower((taxiright + taxiup));
        backright.setPower((-taxiright + taxiup));
    }

    private void brute_force_method(double powerright, double powerup) {
        double magnitude = Math.sqrt(powerright * powerright + powerup * powerup);
        if (magnitude > 1) {
            upleft.setPower((-powerright + powerup) / magnitude);
            backleft.setPower((powerright + powerup) / magnitude);
            upright.setPower((powerright + powerup) / magnitude);
            backright.setPower((-powerright + powerup) / magnitude);
        } else {
            upleft.setPower((-powerright + powerup));
            backleft.setPower((powerright + powerup));
            upright.setPower((powerright + powerup));
            backright.setPower((-powerright + powerup));
        }
    }


    @Override
    public void runOpMode() throws InterruptedException {

        upleft = hardwareMap.dcMotor.get("frontLeft");
        backleft = hardwareMap.dcMotor.get("backLeft");
        backright = hardwareMap.dcMotor.get("backRight");
        upright = hardwareMap.dcMotor.get("frontRight");

        upleft.setDirection(DcMotorSimple.Direction.REVERSE);
        backleft.setDirection(DcMotorSimple.Direction.REVERSE);

        upleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        upright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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
                double adjustable_rotate_factor = .25;

                upleft.setPower(adjustable_rotate_factor * trigger_factor * -rotate_counterclockwise);
                backleft.setPower(adjustable_rotate_factor * trigger_factor * -rotate_counterclockwise);
                upright.setPower(adjustable_rotate_factor * trigger_factor * rotate_counterclockwise);
                backright.setPower(adjustable_rotate_factor * trigger_factor * rotate_counterclockwise);
            } else {
                upleft.setPower(0);
                backleft.setPower(0);
                upright.setPower(0);
                backright.setPower(0);
            }
        }
    }
}
