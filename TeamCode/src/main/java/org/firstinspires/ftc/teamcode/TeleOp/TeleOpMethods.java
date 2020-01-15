package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Exponential_Methods;

@TeleOp(name = "TeleOp: Robot Relative", group = "TeleOp")
public class TeleOpMethods extends Exponential_Methods {
    public static final double LEFT_BUMPER_TRIGGER_FACTOR = .5;
    public static final double RIGHT_BUMPER_TRIGGER_FACTOR = .25;
    public static final double SLIDE_FACTOR = .5;
    public static final double LEFT_SERVO_OPEN_POSITION = .55;
    public static final double Right_SERVO_OPEN_POSITION = .85;
    public static final double LEFT_SERVO_CLOSE_POSITION = .62;
    public static final double Right_SERVO_CLOSE_POSITION = .92;
    public static final int SLIDE_MAX = slideMax - slideMin;
    public static final int SLIDE_MIN = 0;
    public static final double TIMER_INTERVAL = .15;
    public static final double INTAKE_MOTORS_INTAKE = -.45;
    public static final double INTAKE_MOTORS_OUTTAKE = .3;
    public static final double ROTATE_TO_MOVE_RATIO = .8;
    public static final double SLIDE_POWER = .3;

    public ElapsedTime timer = new ElapsedTime();
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        for(DcMotor motor : driveMotors){
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        waitForStart();
        while(opModeIsActive()){
            driveTrain();
            intakeServos();
            hookServos();
            slideMotors();
            intakeMotors();
        }
    }

    public void driveTrain(){
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
    }


    public void intakeServos(){
        //Intake arm servos
        if(gamepad2.b){
            // b button is pressed
            intakeServoLeft.setPosition(LEFT_SERVO_CLOSE_POSITION);
            intakeServoRight.setPosition(Right_SERVO_CLOSE_POSITION);
        }

        if(gamepad2.x){
            // x button is pressed
            intakeServoLeft.setPosition(LEFT_SERVO_OPEN_POSITION);
            intakeServoRight.setPosition(Right_SERVO_OPEN_POSITION);
        }
    }
    public void hookServos(){
        //hook down
        if(gamepad1.x) {
            toggleHook(true);
        }
        //hook up
        if(gamepad1.b) {
            toggleHook(false);
        }
    }

    // moves the slides up and down
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

            // tells the slides to hold their position
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

    // toggles intake
    public void intakeMotors(){
        setIntakeWheels(INTAKE_MOTORS_INTAKE * gamepad2.left_trigger);
        if(gamepad2.right_trigger!=0) {
            setIntakeWheels(INTAKE_MOTORS_OUTTAKE * gamepad2.right_trigger);
        }
    }



    // converts between the input of the trigger to the power of the motors
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
}
