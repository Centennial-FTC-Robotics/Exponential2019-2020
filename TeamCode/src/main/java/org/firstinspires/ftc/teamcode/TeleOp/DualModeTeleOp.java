package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Dual Mode", group = "TeleOp")
public class DualModeTeleOp extends TeleOpMethods {
    String mode = "Regular";
    ElapsedTime time = new ElapsedTime();
    final double TIMER_INTERVAL = 0.3;
    public void driveTrain(){
        if(gamepad1.b&&time.seconds()>TIMER_INTERVAL){
            if(mode.equals("Regular")){
                mode = "Field Centric";
            } else {
                mode = "Regular";
            }
            time.reset();
        }
        if(gamepad1.y){
            resetOrientation();
        }
        if(mode.equals("Regular")){
            double[] answer = circle_to_taxicab(gamepad1.left_stick_x, gamepad1.left_stick_y, ROTATE_TO_MOVE_RATIO*gamepad1.right_stick_x);
            double factor = 1;
            if(gamepad1.left_bumper){
                // if left bumper is pressed, reduce the motor speed
                factor = LEFT_BUMPER_TRIGGER_FACTOR;
            }
            if (gamepad1.right_bumper){
                // if right bumper is pressed, reduce the motor speed
                factor = RIGHT_BUMPER_TRIGGER_FACTOR;
            }
            frontRight.setPower(factor*answer[0]);
            backRight.setPower(factor*answer[1]);
            backLeft.setPower(factor*answer[2]);
            frontLeft.setPower(factor*answer[3]);

            if(gamepad1.dpad_down||gamepad1.dpad_up){
                turnRelative(180);
            }
            if(gamepad1.dpad_right){
                turnRelative(270);
            }
            if(gamepad1.dpad_left){
                turnRelative(90);
            }
        } else {
            double currentAngle = Math.PI*getRotationinDimension('Z')/180;
            // double currentAngle = 0; //  TODO; FIND OUT HOW TO SET LATeR

            //------FIELD CENTRIC-------
            // THESE ANGLES ARE IN STANDARD POSITION. maybe non standard position works, but only if bearing bearing or std. std.
            double inputX = gamepad1.left_stick_x;
            double inputY = -1*gamepad1.left_stick_y;

            double centricX = inputX*Math.cos(currentAngle)+inputY*Math.sin(currentAngle);
            double centricY = inputX*Math.sin(currentAngle)-inputY*Math.cos(currentAngle);

            double[] answer = circle_to_taxicab(centricX, centricY, ROTATE_TO_MOVE_RATIO*gamepad1.right_stick_x);
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
}
