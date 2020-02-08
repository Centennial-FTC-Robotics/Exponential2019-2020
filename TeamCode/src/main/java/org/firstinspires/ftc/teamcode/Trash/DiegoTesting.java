package org.firstinspires.ftc.teamcode.Trash;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.TeleOp.TeleOpMethods;

@TeleOp(group = "TeleOp", name = "Diego Test")
public class DiegoTesting extends TeleOpMethods {

    public static final double LEFT_BUMPER_TRIGGER_FACTOR = .5;
    public static final double RIGHT_BUMPER_TRIGGER_FACTOR = .25;
    public static final double LEFT_SLIDE_FACTOR = .35;
    public static final double RIGHT_SLIDE_FACTOR = .5;
    public static final int SLIDE_MAX_RIGHT = slideMax - slideMin;
    public static final int SLIDE_MAX_LEFT = (int) (SLIDE_MAX_RIGHT * (35.0 / 100.0));

    @Override
    public void runOpMode() throws InterruptedException{
        super.runOpMode();
        if(gamepad1.y){
            resetOrientation();
        }
    }

    public void slideMotors(){
        int slidePosition = (slideUp.getCurrentPosition() + slideDown.getCurrentPosition())/2;
        // height of the slides given from the encoder counts
        if(gamepad2.left_stick_y!=0){
            // user wants to move slides
            slideUp.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            slideDown.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            if(gamepad2.left_bumper){
                // slows down slides according to left bumper
                slideUp.setPower(SLIDE_FACTOR*LEFT_BUMPER_TRIGGER_FACTOR*-gamepad2.left_stick_y);
                slideDown.setPower(SLIDE_FACTOR*LEFT_BUMPER_TRIGGER_FACTOR*-gamepad2.left_stick_y);
            } else if (gamepad2.right_bumper){
                // slows down slides according to right bumper
                slideUp.setPower(RIGHT_BUMPER_TRIGGER_FACTOR*SLIDE_FACTOR*-gamepad2.left_stick_y);
                slideDown.setPower(RIGHT_BUMPER_TRIGGER_FACTOR*SLIDE_FACTOR*-gamepad2.left_stick_y);
            } else {
                // moves slides according to gamepad2 left stick
                slideUp.setPower((SLIDE_FACTOR*-gamepad2.left_stick_y));
                slideDown.setPower((SLIDE_FACTOR*-gamepad2.left_stick_y));
            }
            if(slidePosition > SLIDE_MAX) {
                // slides too high
                if(gamepad2.left_stick_y>0){
                    // if user is pushing the slides down
                    slideUp.setPower(SLIDE_FACTOR*-gamepad2.left_stick_y);
                    slideDown.setPower(SLIDE_FACTOR*-gamepad2.left_stick_y);
                } else {
                    // if user is trying to push the slides up when they have already hit their limit
                    setSlidePower(0);
                }
            } else if(slidePosition < SLIDE_MIN)
                if(gamepad2.left_stick_y<0){
                    // if user wants to push slides back up
                    slideUp.setPower(SLIDE_FACTOR*-gamepad2.left_stick_y);
                    slideDown.setPower(SLIDE_FACTOR*-gamepad2.left_stick_y);
                } else {
                    // if user is trying to push the slides down when they have already hit their limit
                    setSlidePower(0);
                }
        } else {
            // the user is not trying to move slides
            if(slidePosition > SLIDE_MAX) {
                // if the slide have exceeded the height, go back to slide max
                slidePosition = SLIDE_MAX;
            } else if(slidePosition < SLIDE_MIN) {
                // if the slide is too low, go back to slide min
                slidePosition = SLIDE_MIN;
            }

            if ((slidePosition - (slideUp.getCurrentPosition() + slideDown.getCurrentPosition())/2) > 0) {

                setZeroPowerMode(true);
            } else if ((slidePosition - (slideUp.getCurrentPosition() + slideDown.getCurrentPosition())/2) < 0) {

                setZeroPowerMode(false);
            }

            // tells the slides to hold the position that the slides were at when the user released the trigger
            slideUp.setTargetPosition(slidePosition);
            slideDown.setTargetPosition(slidePosition);
            slideUp.setPower(SLIDE_POWER);
            slideDown.setPower(SLIDE_POWER);
            slideUp.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slideDown.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        if(gamepad2.y&&timer.seconds()>TIMER_INTERVAL){
            // if button y is pressed, move slides up 4 inches
            slidePosition += convertInchToEncoderSlides(4);
            timer.reset();
        }
        if(gamepad2.a&&timer.seconds()>TIMER_INTERVAL){
            // if button a is pressed, move slides down 4 inches
            slidePosition -= convertInchToEncoderSlides(4);
            timer.reset();
        }
    }

    // moves the slides up and down
//    public void slideMotors(){
//        int slidePositionRight = slideUp.getCurrentPosition();
//        int slidePositionLeft = slideDown.getCurrentPosition();
//
//        // height of the slides given from the encoder counts
//        if(gamepad2.left_stick_y!=0){
//            // user wants to move slides
//            slideUp.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            slideDown.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            if(gamepad2.left_bumper){
//                // slows down slides according to left bumper
//                slideUp.setPower(RIGHT_SLIDE_FACTOR *LEFT_BUMPER_TRIGGER_FACTOR*-gamepad2.left_stick_y);
//                slideDown.setPower(LEFT_SLIDE_FACTOR *LEFT_BUMPER_TRIGGER_FACTOR*-gamepad2.left_stick_y);
//            } else if (gamepad2.right_bumper){
//                // slows down slides according to right bumper
//                slideUp.setPower(RIGHT_BUMPER_TRIGGER_FACTOR* RIGHT_SLIDE_FACTOR *-gamepad2.left_stick_y);
//                slideDown.setPower(RIGHT_BUMPER_TRIGGER_FACTOR* LEFT_SLIDE_FACTOR *-gamepad2.left_stick_y);
//            } else {
//                // moves slides according to gamepad2 left stick
//                slideUp.setPower((RIGHT_SLIDE_FACTOR *-gamepad2.left_stick_y));
//                slideDown.setPower((LEFT_SLIDE_FACTOR *-gamepad2.left_stick_y));
//            }
//
//            if(slidePositionRight > SLIDE_MAX_RIGHT) {
//                // slides too high
//                if(gamepad2.left_stick_y>0){
//                    // if user is pushing the slides down
//                    slideUp.setPower(RIGHT_SLIDE_FACTOR *-gamepad2.left_stick_y);
//                } else {
//                    // if user is trying to push the slides up when they have already hit their limit
//                    setSlidePowRight(0);
//                }
//            } else if(slidePositionRight < SLIDE_MIN) {
//                if (gamepad2.left_stick_y < 0) {
//                    // if user wants to push slides back up
//                    slideUp.setPower(RIGHT_SLIDE_FACTOR * -gamepad2.left_stick_y);
//                } else {
//                    // if user is trying to push the slides down when they have already hit their limit
//                    setSlidePowRight(0);
//                }
//            }
//
//            if(slidePositionLeft > SLIDE_MAX_LEFT) {
//                // slides too high
//                if(gamepad2.left_stick_y>0){
//                    // if user is pushing the slides down
//                    slideDown.setPower(LEFT_SLIDE_FACTOR *-gamepad2.left_stick_y);
//                } else {
//                    // if user is trying to push the slides up when they have already hit their limit
//                    setSlidePowLeft(0);
//                }
//            } else if(slidePositionLeft < SLIDE_MIN) {
//                if (gamepad2.left_stick_y < 0) {
//                    // if user wants to push slides back up
//                    slideDown.setPower(LEFT_SLIDE_FACTOR * -gamepad2.left_stick_y);
//                } else {
//                    // if user is trying to push the slides down when they have already hit their limit
//                    setSlidePowLeft(0);
//                }
//            }
//        } else {
//            // the user is not trying to move slides
//            if(slidePositionRight > SLIDE_MAX_RIGHT) {
//                // if the slide have exceeded the height, go back to slide max
//                slidePositionRight = SLIDE_MAX_RIGHT;
//            } else if(slidePositionRight < SLIDE_MIN) {
//                // if the slide is too low, go back to slide min
//                slidePositionRight = SLIDE_MIN;
//            }
//
//            if(slidePositionLeft > SLIDE_MAX_LEFT) {
//                // if the slide have exceeded the height, go back to slide max
//                slidePositionLeft = SLIDE_MAX_LEFT;
//            } else if(slidePositionLeft < SLIDE_MIN) {
//                // if the slide is too low, go back to slide min
//                slidePositionLeft = SLIDE_MIN;
//            }
//
//            // tells the slides to hold the position that the slides were at when the user released the trigger
//
//            if ((slidePositionLeft - slideDown.getCurrentPosition()) > 0) {
//
//                setZeroPowerMode(true);
//            } else if ((slidePositionLeft - slideDown.getCurrentPosition()) < 0) {
//
//                setZeroPowerMode(false);
//            }
//
//            slideUp.setTargetPosition(slidePositionRight);
//            slideDown.setTargetPosition(slidePositionLeft);
//            slideUp.setPower(SLIDE_POWER);
//            slideDown.setPower(SLIDE_POWER);
//            slideUp.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            slideDown.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        }
//        if(gamepad2.y&&timer.seconds()>TIMER_INTERVAL){
//            // if button y is pressed, move slides up 4 inches
//            slidePositionRight += convertInchToEncoderSlides(4);
//            slidePositionLeft += convertInchToEncoderSlides(4);
//            timer.reset();
//        }
//        if(gamepad2.a&&timer.seconds()>TIMER_INTERVAL){
//            // if button a is pressed, move slides down 4 inches
//            slidePositionRight -= convertInchToEncoderSlides(4);
//            slidePositionLeft -= convertInchToEncoderSlides(4);
//            timer.reset();
//        }
//    }

    public void setSlidePowRight(double power) {

        slideUp.setPower(0);
    }

    public void setSlidePowLeft(double power) {

        slideDown.setPower(0);
    }

    public void setZeroPowerMode(boolean up) {
        if (up) {
            slideUp.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            slideDown.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        } else {
            slideUp.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            slideDown.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }
}
