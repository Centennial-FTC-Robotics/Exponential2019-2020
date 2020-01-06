package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Hardware;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.internal.tfod.Timer;

@TeleOp(name = "TeleOp, USE THIS ONE", group = "TeleOp")

public class TeleOpDriver extends Exponential_Methods {
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



        for(DcMotor motor : driveMotors){
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        waitForStart();


        int slidePosition = (slideDown.getCurrentPosition())/2;

        while (opModeIsActive()) {
            telemetry.addData("Y Value", gamepad2.left_stick_y);
            telemetry.addData("Position", slidePosition);
            //telemetry.update();
            if(gamepad2.left_stick_y!=0){
                slideUp.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                slideDown.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                if(gamepad2.left_bumper){
                    setSlidePower(.25*.3*-gamepad2.left_stick_y);
                } else if (gamepad2.right_bumper){
                    setSlidePower(.5*.3*-gamepad2.left_stick_y);
                } else {
                    setSlidePower(.3*-gamepad2.left_stick_y);
                }
                slidePosition = (slideDown.getCurrentPosition());
            } else {
                if(slidePosition > slideMax)
                    slidePosition = slideMax;
                else if(slidePosition < slideMin)
                    slidePosition = slideMin;
                slideUp.setTargetPosition(slidePosition);
                slideDown.setTargetPosition(slidePosition);
                slideUp.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slideDown.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            telemetry.update();
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

            //hook down
            if(gamepad1.x)
                toggleHook(true);

            //hook up
            if(gamepad1.b)
                toggleHook(false);

            //slide speed reduced to 1/2
            //if(gamepad2.left_bumper)
                //setSlidePower(slideUp.getPower() / 2);

            //slide speed reduced to 1/4
            //if(gamepad2.right_bumper)
                //setSlidePower(slideUp.getPower() / 4);

            double intakePower = .9; //set later
            setIntakeWheels(intakePower * gamepad2.left_trigger);
            if(gamepad2.right_trigger!=0) {
                setIntakeWheels(-.25 * intakePower * gamepad2.right_trigger);
            }
            //Make slides go up 4 inches
            if(gamepad2.y){
                slidePosition += convertInchToEncoderSlides(4);
            }
            if(gamepad2.a){
                slidePosition -= convertInchToEncoderSlides(4);
            }

            //Intake arm servos
            if(gamepad2.b){
                intakeServoLeft.setPosition(.92);
                intakeServoRight.setPosition(.95);
            }

            if(gamepad2.x){
                //intake servos release stone
                intakeServoRight.setPosition(.7);
                intakeServoLeft.setPosition(.65);
            }
        }
    }

}
