package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Hardware;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.internal.tfod.Timer;

@TeleOp(name = "teleop slides still up", group = "TeleOp")

public class TeleOpDriverSlidesStillUp extends TeleOpMethods {

    public final static int SLIDE_MAX = slideMax;
    public final static int SLIDE_MIN = slideMin;

    @Override
    public void slideMotors() {
        int slidePosition = (slideUp.getCurrentPosition() + slideDown.getCurrentPosition())/2;
        if(gamepad2.left_stick_y!=0){
            slideUp.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            slideDown.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            if(gamepad2.left_bumper){
                slideUp.setPower(SLIDE_FACTOR*LEFT_BUMPER_TRIGGER_FACTOR*-gamepad2.left_stick_y);
                slideDown.setPower(SLIDE_FACTOR*LEFT_BUMPER_TRIGGER_FACTOR*-gamepad2.left_stick_y);
            } else if (gamepad2.right_bumper){
                slideUp.setPower(RIGHT_BUMPER_TRIGGER_FACTOR*SLIDE_FACTOR*-gamepad2.left_stick_y);
                slideDown.setPower(RIGHT_BUMPER_TRIGGER_FACTOR*SLIDE_FACTOR*-gamepad2.left_stick_y);
            } else {
                slideUp.setPower((SLIDE_FACTOR*-gamepad2.left_stick_y));
                slideDown.setPower((SLIDE_FACTOR*-gamepad2.left_stick_y));
            }
            if(slidePosition > SLIDE_MAX) {
                if(gamepad2.left_stick_y>0){
                    slideUp.setPower(SLIDE_FACTOR*-gamepad2.left_stick_y);
                    slideDown.setPower(SLIDE_FACTOR*-gamepad2.left_stick_y);
                } else {
                    setSlidePower(0);
                }
            } else if(slidePosition < SLIDE_MIN)
                if(gamepad2.left_stick_y>0){
                    slideUp.setPower(SLIDE_FACTOR*-gamepad2.left_stick_y);
                    slideDown.setPower(SLIDE_FACTOR*-gamepad2.left_stick_y);
                } else {
                    setSlidePower(0);
                }
            //else
            //slidePosition = (slideDown.getCurrentPosition() + slideUp.getCurrentPosition()) / 2;
        } else {
            if(slidePosition > SLIDE_MAX)
                slidePosition = SLIDE_MAX;
            else if(slidePosition < SLIDE_MIN)
                slidePosition = SLIDE_MIN;
            slideUp.setTargetPosition(slidePosition);
            slideDown.setTargetPosition(slidePosition);
            slideUp.setPower(SLIDE_POWER);
            slideDown.setPower(SLIDE_POWER);
            slideUp.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slideDown.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        if(gamepad2.y&&timer.seconds()>TIMER_INTERVAL){
            slidePosition += convertInchToEncoderSlides(4);
            timer.reset();
        }
        if(gamepad2.a&&timer.seconds()>TIMER_INTERVAL){
            slidePosition -= convertInchToEncoderSlides(4);
            timer.reset();
        }
    }
    }
}