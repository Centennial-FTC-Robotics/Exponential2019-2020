package org.firstinspires.ftc.teamcode.TeleOp;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.TeleOpMethods;

@TeleOp(name = "TeleOp: OneController", group = "TeleOp")
public class OneController extends TeleOpMethods {
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
    }

    public void driveTrain(){
        double[] answer = getMotorPowers(gamepad1.left_stick_x, -gamepad1.left_stick_y, ROTATE_TO_MOVE_RATIO*gamepad1.right_stick_x);
        double factor = .9;

        frontRight.setPower(factor*answer[0]);
        backRight.setPower(factor*answer[1]);
        backLeft.setPower(factor*answer[2]);
        frontLeft.setPower(factor*answer[3]);
    }

    public void intakeServos() {
        //Intake arm servos
        if (gamepad1.x && intakeTimer.seconds() > INTAKE_SERVOS_TIMER_INTERVAL) {
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

    public void hookServos(){
        if(gamepad1.a && hookTimer.seconds()>HOOK_SERVOS_TIMER_INTERVAL){
            hooksDown=!hooksDown;
            hookTimer.reset();
        }
        toggleHook(hooksDown);
    }

    public void yeetServos(){
        if(gamepad1.y){
            yeetServo.setPosition(YEETER_EXTENSION_POSITION);
        }
    }

    public void hoodServos(){
        if(gamepad1.left_bumper){
            hoodDown=!hoodDown;
        }
        //toggleHood(hoodDown);
    }

    public void slideMotors(){
        double slideSpeed = 0.5;
        if(gamepad1.dpad_up && slidePosition < SLIDE_MAX){
            slideUp.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            slideDown.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            slidePosition = (slideUp.getCurrentPosition());
            slideUp.setPower(slideSpeed);
            slideDown.setPower(slideSpeed);
        } else if(gamepad1.dpad_down && slidePosition > SLIDE_MIN){
            slideUp.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            slideDown.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            slidePosition = (slideUp.getCurrentPosition());
            slideUp.setPower(-slideSpeed);
            slideDown.setPower(-slideSpeed);
        } else {
            // the user is not trying to move slides
            if(slidePosition > SLIDE_MAX) {
                // if the slide have exceeded the height, go back to slide max
                slidePosition = SLIDE_MAX;
            } else if(slidePosition < SLIDE_MIN) {
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
        telemetry.addData("Up Slide", slideUp.getCurrentPosition());
        telemetry.addData("Down Slide", slideDown.getCurrentPosition());
        telemetry.update();
    }

    // toggles intake wheels
    public void intakeMotors(){
        setIntakeWheels(INTAKE_MOTORS_INTAKE * gamepad1.left_trigger);
        if (gamepad1.right_trigger!=0)
            setIntakeWheels(INTAKE_MOTORS_OUTTAKE * gamepad1.right_trigger);
        if(gamepad1.right_stick_y!=0){
            setIntakeWheels(gamepad1.right_stick_y*INTAKE_WHEELS_SPEED_FACTOR);
        }
    }
}
