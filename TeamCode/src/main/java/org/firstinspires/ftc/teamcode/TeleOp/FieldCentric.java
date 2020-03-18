package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.TeleOpMethods;

@TeleOp(name = "FieldCentric", group = "TeleOp")

public class FieldCentric extends TeleOpMethods {
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();

    }
    public void driveTrain() {
        if(gamepad1.y){
            resetOrientation();
        }
        double currentAngle = Math.PI* getRotationInDimension('Z')/180;
        // double currentAngle = 0; //  TODO; FIND OUT HOW TO SET LATeR

        //------FIELD CENTRIC-------
        // THESE ANGLES ARE IN STANDARD POSITION. maybe non standard position works, but only if bearing bearing or std. std.
        double inputX = gamepad1.left_stick_x;
        double inputY = -1*gamepad1.left_stick_y;

        double centricX = inputX*Math.cos(currentAngle)+inputY*Math.sin(currentAngle);
        double centricY = inputX*Math.sin(currentAngle)-inputY*Math.cos(currentAngle);

        double[] answer = getMotorPowers(centricX, centricY, ROTATE_TO_MOVE_RATIO*gamepad1.right_stick_x);
        double factor = 1;
        if(gamepad1.left_bumper){
            factor = LEFT_BUMPER_TRIGGER_FACTOR;
        }
        if (gamepad1.right_bumper){
            factor = RIGHT_BUMPER_TRIGGER_FACTOR;
        }
        frontRight.setPower(factor*answer[0]);
        backRight.setPower(factor*answer[1]);
        backLeft.setPower(factor*answer[2]);
        frontLeft.setPower(factor*answer[3]);
    }
}