package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Hardware;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.internal.tfod.Timer;


public class TeleOpDriver extends Exponential_Methods {

    public final static int SLIDE_MAX = slideMax - slideMin;
    public final static int SLIDE_MIN = 0;
    private double[] circle_to_taxicab(double circle_x, double circle_y, double circle_rotate) {
        double[] answer = new double[4];
        double x;
        double y;

        if (circle_x == 0.0) {
            x = 0.0;
        }else {
            x = circle_x / Math.abs(circle_x) * Math.sqrt(Math.pow(circle_x, 2) + Math.pow(circle_y, 2))
                    * (Math.abs(circle_x)) / (Math.abs(circle_x) + Math.abs(circle_y));
        }
        if (circle_y == 0.0) {
            y = 0.0;
        }else {
            y = circle_y / Math.abs(circle_y) * Math.sqrt(Math.pow(circle_x, 2) + Math.pow(circle_y, 2))
                    * (Math.abs(circle_y)) / (Math.abs(circle_x) + Math.abs(circle_y));
        }
        double sum = Math.abs(x)+Math.abs(y)+Math.abs(circle_rotate);
        if(sum >1){
            answer[0]=(x+y+circle_rotate)/sum;
            answer[1]=(-x+y+circle_rotate)/sum;
            answer[2]=(x+y-circle_rotate)/sum;
            answer[3]=(-x+y-circle_rotate)/sum;
        } else {
            answer[0]=(x+y+circle_rotate);
            answer[1]=(-x+y+circle_rotate);
            answer[2]=(x+y-circle_rotate);
            answer[3]=(-x+y-circle_rotate);
        }
        return answer;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();



        waitForStart();


        int slidePosition = (slideUp.getCurrentPosition() + slideDown.getCurrentPosition())/2;
        ElapsedTime timer = new ElapsedTime();

        //-----------------------------------SLIDES------------------------------------
        while (opModeIsActive()) {
            telemetry.addData("SlideDown", slideDown.getCurrentPosition());
            telemetry.addData("SlideUp", slideUp.getCurrentPosition());
            telemetry.addData("SlidePosition", slidePosition);
            telemetry.update();
            if(gamepad2.left_stick_y!=0){
                slideUp.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                slideDown.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                if(gamepad2.left_bumper){
                    slideUp.setPower(.25*.5*-gamepad2.left_stick_y);
                    slideDown.setPower(.25*.5*-gamepad2.left_stick_y);
                } else if (gamepad2.right_bumper){
                    slideUp.setPower(.5*.5*-gamepad2.left_stick_y);
                    slideDown.setPower(.5*.5*-gamepad2.left_stick_y);
                } else {
                    slideUp.setPower((.5*-gamepad2.left_stick_y));
                    slideDown.setPower((.5*-gamepad2.left_stick_y));
                }
                if(slidePosition > SLIDE_MAX) {
                    if(gamepad2.left_stick_y>0){
                        slideUp.setPower(.5*-gamepad2.left_stick_y);
                        slideDown.setPower(.5*-gamepad2.left_stick_y);
                    } else {
                        setSlidePower(0);
                    }
                } else if(slidePosition < SLIDE_MIN)
                    if(gamepad2.left_stick_y>0){
                        slideUp.setPower(.25*.5*-gamepad2.left_stick_y);
                        slideDown.setPower(.25*.5*-gamepad2.left_stick_y);
                    } else {
                        setSlidePower(0);
                    }
                else
                    slidePosition = (slideDown.getCurrentPosition() + slideUp.getCurrentPosition()) / 2;
            } else {
                if(slidePosition > SLIDE_MAX)
                    slidePosition = SLIDE_MAX;
                else if(slidePosition < SLIDE_MIN)
                    slidePosition = SLIDE_MIN;
                slideUp.setTargetPosition(slidePosition);
                slideDown.setTargetPosition(slidePosition);
                slideUp.setPower(.3);
                slideDown.setPower(.3);
                slideUp.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slideDown.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            //Make slides go up 4 inches
            if(gamepad2.y&&timer.seconds()>.25){
                slidePosition += convertInchToEncoderSlides(4);
                timer = new ElapsedTime();
            }
            if(gamepad2.a&&timer.seconds()>.25){
                slidePosition -= convertInchToEncoderSlides(4);
                timer = new ElapsedTime();
            }
            //-----------------------------------SPEED------------------------------------

            double bumper_factor = 1;
            double left_trigger_factor = gamepad1.left_trigger;
            double right_trigger_factor = gamepad1.right_trigger;

            //double bumper_factor = 1.0 - gamepad1.left_trigger;
            double[] answer = circle_to_taxicab(gamepad1.left_stick_x, gamepad1.left_stick_y, .8*gamepad1.right_stick_x);
            if(gamepad1.left_bumper){
                bumper_factor = .5;
            }
            if (gamepad1.right_bumper){
                bumper_factor = .25;
            }

            frontRight.setPower(bumper_factor*answer[0]);
            backRight.setPower(bumper_factor*answer[1]);
            backLeft.setPower(bumper_factor*answer[2]);
            frontLeft.setPower(bumper_factor*answer[3]);

            //-----------------------------------HOOKS------------------------------------

            //hook down
            if(gamepad1.x&&timer.seconds()>.25) {
                toggleHook(true);
                timer = new ElapsedTime();
            }
            //hook up
            if(gamepad1.b&&timer.seconds()>.25) {
                toggleHook(false);
                timer = new ElapsedTime();
            }

            //-----------------------------------INTAKE WHEELS------------------------------------
            double intakePower = .9; //set later
            setIntakeWheels(intakePower * gamepad2.left_trigger);
            if(gamepad2.right_trigger!=0) {
                setIntakeWheels(-.5 * intakePower * gamepad2.right_trigger);
            }

            //-----------------------------------INTAKE ARMS------------------------------------
            //Intake arm servos
            if(gamepad2.b&&timer.seconds()>.25){
                intakeServoLeft.setPosition(.62);
                intakeServoRight.setPosition(.92);
                timer = new ElapsedTime();
            }

            if(gamepad2.x&&timer.seconds()>.25){
                //intake servos release stone
                intakeServoLeft.setPosition(.55);
                intakeServoRight.setPosition(.85);
                timer = new ElapsedTime();
            }
        }
    }

}