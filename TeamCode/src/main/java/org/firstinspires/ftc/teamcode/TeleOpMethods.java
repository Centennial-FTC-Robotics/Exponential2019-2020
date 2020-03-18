package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.internal.tfod.Timer;
import org.firstinspires.ftc.teamcode.Exponential_Methods;

@TeleOp(name = "TeleOp: Regular", group = "TeleOp")
public class TeleOpMethods extends Exponential_Methods {
    // The amount the drive train and slides slow down to when the left and right bumper is held down respectively
    public static final double LEFT_BUMPER_TRIGGER_FACTOR = .5;
    public static final double RIGHT_BUMPER_TRIGGER_FACTOR = .25;


    // The servo positions of the intake servos when they open and close
    public static final double LEFT_SERVO_OPEN_POSITION = .4;
    public static final double RIGHT_SERVO_OPEN_POSITION = .7;
    public static final double LEFT_SERVO_CLOSE_POSITION = .66;
    public static final double RIGHT_SERVO_CLOSE_POSITION = .96;

    // Max and minimum positions of the slides (slideMax and slideMin are variables in exponential methods)
    public static final int SLIDE_MAX = slideMax - slideMin;
    public static final int SLIDE_MIN = 0;

    // To prevent double pressing, we added a timers that would only render a button press ever TIMER_INTERVAL seconds
    public static final double TIMER_INTERVAL = .15;
    public static final double INTAKE_SERVOS_TIMER_INTERVAL = .3;
    public static final double HOOK_SERVOS_TIMER_INTERVAL = .3;
    public static final double HOOD_SERVOS_TIMER_INTERVAL = .3;
    public static final double INTAKE_MOTORS_INTAKE = -1;
    public static final double INTAKE_MOTORS_OUTTAKE = 1;

    // Adjusts how sensitive the rotation of a robot is compared to its linear movement (larger the ratio, the more sensitive the rotation will be)
    public static final double ROTATE_TO_MOVE_RATIO = .8;

    // Adjusts how fast or slow the slide motors go
    public static final double SLIDE_FACTOR = .63;

    // Adjusts the speed of the intake wheels
    public static final double INTAKE_WHEELS_SPEED_FACTOR = 1;

    // Don't worry about this
    public static final double SLIDE_POWER = .3; //Don't change this, EVER

    // Position of the yeeter servo
    public static final double YEETER_EXTENSION_POSITION = .4;

    public boolean servosOpen = true;
    public boolean hooksDown = false;
    public boolean hoodDown = false;
    public boolean clawDown = false;
    public ElapsedTime clawTimer = new ElapsedTime();
    public ElapsedTime intakeTimer = new ElapsedTime();
    public ElapsedTime hookTimer = new ElapsedTime();
    public ElapsedTime timer = new ElapsedTime();
    public ElapsedTime hoodTimer = new ElapsedTime();

    public int slidePosition = 0;

    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        for (DcMotor motor : driveMotors) {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        waitForStart();

        while (opModeIsActive()) {
            driveTrain();
            intakeServos();
            hookServos();
            slideMotors();
            intakeMotors();
            yeetServos();
            clawServo();
            //hoodServos();
            decay.updateRobot();
            telemetry.addData("x coordinate in inches", convertEncoderToInchOdom(decay.currentX));
            telemetry.addData("y coordinate in inches", convertEncoderToInchOdom(decay.currentY));
            telemetry.addData("angle", decay.currentAngle);
            telemetry.update();
        }
    }

    public void driveTrain() {
        double[] motorPowers = getMotorPowers(gamepad1.left_stick_x, -gamepad1.left_stick_y, ROTATE_TO_MOVE_RATIO * gamepad1.right_stick_x);
        double factor = 1;
        if (gamepad1.left_bumper) {
            // if left bumper is pressed, reduce the motor speed
            factor = LEFT_BUMPER_TRIGGER_FACTOR;
        }
        if (gamepad1.right_bumper) {
            // if right bumper is pressed, reduce the motor speed
            factor = RIGHT_BUMPER_TRIGGER_FACTOR;
        }
        frontRight.setPower(factor * motorPowers[0]);
        backRight.setPower(factor * motorPowers[1]);
        backLeft.setPower(factor * motorPowers[2]);
        frontLeft.setPower(factor * motorPowers[3]);

        // D Pad driver assist
        if (gamepad1.dpad_down || gamepad1.dpad_up) {
            turnRelative(180);
        }
        if (gamepad1.dpad_right) {
            turnRelative(270);
        }
        if (gamepad1.dpad_left) {
            turnRelative(90);
        }
    }


    public void intakeServos() {
        //Intake arm servos
        if (gamepad2.x && intakeTimer.seconds() > INTAKE_SERVOS_TIMER_INTERVAL) {
            servosOpen = !servosOpen;
            intakeTimer.reset();
        }
        if (servosOpen) {
            intakeServoLeft.setPosition(LEFT_SERVO_OPEN_POSITION);
            intakeServoRight.setPosition(RIGHT_SERVO_OPEN_POSITION);
        } else {
            intakeServoLeft.setPosition(LEFT_SERVO_CLOSE_POSITION);
            intakeServoRight.setPosition(RIGHT_SERVO_CLOSE_POSITION);
        }
    }

    public void hookServos() {
        if (gamepad1.x && hookTimer.seconds() > HOOK_SERVOS_TIMER_INTERVAL) {
            hooksDown = !hooksDown;
            hookTimer.reset();
        }
        toggleHook(hooksDown);
    }

    public void clawServo(){
        if (gamepad2.b && clawTimer.seconds() > HOOK_SERVOS_TIMER_INTERVAL) {
            clawDown = !clawDown;
            clawTimer.reset();
        }
        toggleClaw(clawDown);
    }

    // yeeter
    public void yeetServos() {
        //extend
        if (gamepad1.b) {
            //yeeter methods
            yeetServo.setPosition(YEETER_EXTENSION_POSITION);
        }
    }

    public void hoodServos() {
        if (gamepad2.y && hoodTimer.seconds() > HOOD_SERVOS_TIMER_INTERVAL) {
            hoodDown = !hoodDown;
            hoodTimer.reset();
        }
        //toggleHood(hoodDown);
    }

    // moves the slides up and down
    public void slideMotors() {
        // height of the slides given from the encoder counts
        if (gamepad2.left_stick_y != 0) {
            slidePosition = (slideUp.getCurrentPosition());
            // user wants to move slides
            slideUp.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            slideDown.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            if (gamepad2.left_bumper) {
                // slows down slides according to left bumper
                slideUp.setPower(SLIDE_FACTOR * LEFT_BUMPER_TRIGGER_FACTOR * -gamepad2.left_stick_y);
                slideDown.setPower(SLIDE_FACTOR * LEFT_BUMPER_TRIGGER_FACTOR * -gamepad2.left_stick_y);
            } else if (gamepad2.right_bumper) {
                // slows down slides according to right bumper
                slideUp.setPower(RIGHT_BUMPER_TRIGGER_FACTOR * SLIDE_FACTOR * -gamepad2.left_stick_y);
                slideDown.setPower(RIGHT_BUMPER_TRIGGER_FACTOR * SLIDE_FACTOR * -gamepad2.left_stick_y);
            } else {
                // moves slides according to gamepad2 left stick
                slideUp.setPower((SLIDE_FACTOR * -gamepad2.left_stick_y));
                slideDown.setPower((SLIDE_FACTOR * -gamepad2.left_stick_y));
            }
            if (slidePosition > SLIDE_MAX) {
                // slides too high
                if (gamepad2.left_stick_y > 0) {
                    // if user is pushing the slides down
                    slideUp.setPower(SLIDE_FACTOR * -gamepad2.left_stick_y);
                    slideDown.setPower(SLIDE_FACTOR * -gamepad2.left_stick_y);
                } else {
                    // if user is trying to push the slides up when they have already hit their limit
                    setSlidePower(0);
                }
            } else if (slidePosition < SLIDE_MIN)
                if (gamepad2.left_stick_y < 0) {
                    // if user wants to push slides back up
                    slideUp.setPower(SLIDE_FACTOR * -gamepad2.left_stick_y);
                    slideDown.setPower(SLIDE_FACTOR * -gamepad2.left_stick_y);
                } else {
                    // if user is trying to push the slides down when they have already hit their limit
                    setSlidePower(0);
                }
        } else {
            // the user is not trying to move slides
            if (slidePosition > SLIDE_MAX) {
                // if the slide have exceeded the height, go back to slide max
                slidePosition = SLIDE_MAX;
            } else if (slidePosition < SLIDE_MIN) {
                // if the slide is too low, go back to slide min
                slidePosition = SLIDE_MIN;
            }

            // tells the slides to hold the position that the slides were at when the user released the trigger
            slideUp.setTargetPosition(slidePosition);
            slideDown.setTargetPosition(slidePosition);
            slideUp.setPower(SLIDE_POWER);
            slideDown.setPower(SLIDE_POWER);
            slideUp.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slideDown.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        //telemetry.addData("Up Slide", slideUp.getCurrentPosition());
        //telemetry.addData("Down Slide", slideDown.getCurrentPosition());
        //telemetry.update();
    }

    // toggles intake wheels
    public void intakeMotors() {
        setIntakeWheels(INTAKE_MOTORS_INTAKE * gamepad2.left_trigger);
        if (gamepad2.right_trigger != 0)
            setIntakeWheels(INTAKE_MOTORS_OUTTAKE * gamepad2.right_trigger);
        if (gamepad2.right_stick_y != 0) {
            setIntakeWheels(gamepad2.right_stick_y * INTAKE_WHEELS_SPEED_FACTOR);
        }
    }
}