package org.firstinspires.ftc.teamcode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.TeleOpMethods;

@TeleOp(name = "TeleOp: Kid Friendly", group = "TeleOp")
public class KidFriendly extends TeleOpMethods {
    public void driveTrain() {
        double[] answer = getMotorPowers(gamepad1.left_stick_x, -gamepad1.left_stick_y, ROTATE_TO_MOVE_RATIO * gamepad1.right_stick_x);
        double factor = 1;
        if (gamepad1.left_bumper) {
            // if left bumper is pressed, reduce the motor speed
            factor = LEFT_BUMPER_TRIGGER_FACTOR;
        }
        if (gamepad1.right_bumper) {
            // if right bumper is pressed, reduce the motor speed
            factor = RIGHT_BUMPER_TRIGGER_FACTOR;
        }
        factor = .35;
        frontRight.setPower(factor * answer[0]);
        backRight.setPower(factor * answer[1]);
        backLeft.setPower(factor * answer[2]);
        frontLeft.setPower(factor * answer[3]);
    }
}