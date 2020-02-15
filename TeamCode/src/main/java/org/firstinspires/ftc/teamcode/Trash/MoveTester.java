package org.firstinspires.ftc.teamcode.Trash;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Exponential_Methods;

@TeleOp(name = "MoveTester", group = "TeleOp")

public class MoveTester extends Exponential_Methods {

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        double sideways = 10;
        double forwards = 0;
        telemetry.addData("It got up here", "hello");
        telemetry.update();
        waitForStart();
        while(opModeIsActive()) {
            if(gamepad1.left_stick_x!=0||gamepad1.left_stick_y!=0||gamepad1.right_stick_x!=0||gamepad1.right_stick_y!=0) {
                sideways += .25*(int)(gamepad1.left_stick_x);
                forwards -= .25*(int)(gamepad1.left_stick_y);
                // p+=0.00001*(int)(gamepad1.right_stick_x);
                // i-=0.0001*(int)(gamepad1.right_stick_y);
                sleep(200);
            }
            telemetry.addData("forwards", forwards);
            telemetry.addData("sideways", sideways);
            // telemetry.addData("p value", p);
            // telemetry.addData("i value", i);

            // telemetry.update();
            if(gamepad1.a){
                move(sideways, forwards, .5, .5);
            }
            telemetry.update();
        }
    }
    public void move(double inchesSideways, double inchesForward, double maxPower, double inchesTolerance){  // DON'T FUCK WITH THIS METHOD, i will find a better way to do this later
        double p;
        double i;
        inchesForward = -inchesForward;
        inchesSideways = getTransformedDistance(inchesSideways);

        if(inchesSideways!=0) {
            p = 1.0/600;
            if (Math.sqrt(inchesSideways * inchesSideways + inchesForward * inchesForward) < 5) {
                i = 0.01;
            } else {
                i=0.009;
            }
        } else {
            p=0;//1.0/1200;
            if (Math.sqrt(inchesSideways * inchesSideways + inchesForward * inchesForward) < 5){
                i = 0.005;
            } else {
                i = 0;
            }
        }
        double d = 0;
        double max_positive = maxPower;
        double min_negative = -maxPower;

        double encoderForward = convertInchToEncoder(inchesForward);
        double encoderSideways = convertInchToEncoder(inchesSideways);
        resetDriveMotorEncoders();
        double tolerance = convertInchToEncoder(inchesTolerance);

        double frontLeft_encoder = encoderForward-encoderSideways;
        double frontRight_encoder = encoderForward+encoderSideways;
        double backLeft_encoder = encoderForward+encoderSideways;
        double backRight_encoder = encoderForward-encoderSideways;

        double frontLeft_displacement = frontLeft_encoder-frontLeft.getCurrentPosition();
        double frontRight_displacement = frontRight_encoder-frontRight.getCurrentPosition();
        double backLeft_displacement = backLeft_encoder-backLeft.getCurrentPosition();
        double backRight_displacement = backRight_encoder-backRight.getCurrentPosition();
        ElapsedTime time = new ElapsedTime();

        double areaFrontLeft= 0;
        double areaFrontRight= 0;
        double areaBackLeft= 0;
        double areaBackRight= 0;

        double frontLeftLastPosition = 0;
        double frontRightLastPosition = 0;
        double backLeftLastPosition = 0;
        double backRightLastPosition = 0;

        while (opModeIsActive()&&(Math.abs(frontLeft_displacement)>tolerance||Math.abs(frontRight_displacement)>tolerance||Math.abs(backLeft_displacement)>tolerance||Math.abs(backRight_displacement)>tolerance)){
            areaFrontLeft+=time.seconds()*frontLeft_displacement;
            areaFrontRight+=time.seconds()*frontRight_displacement;
            areaBackLeft+=time.seconds()*backLeft_displacement;
            areaBackRight+=time.seconds()*backRight_displacement;
            double speedFrontLeft= (frontLeft.getCurrentPosition()-frontLeftLastPosition)/time.seconds();
            double speedFrontRight= (frontRight.getCurrentPosition()-frontRightLastPosition)/time.seconds();
            double speedBackLeft= (backLeft.getCurrentPosition()-backLeftLastPosition)/time.seconds();
            double speedBackRight= (backRight.getCurrentPosition()-backRightLastPosition)/time.seconds();
            //if(time.seconds()>2){
            frontLeft.setPower(Range.clip(p*frontLeft_displacement+i*areaFrontLeft+d*speedFrontLeft, min_negative, max_positive));
            frontRight.setPower(Range.clip(p*frontRight_displacement+i*areaFrontRight+d*speedFrontRight, min_negative, max_positive));
            backLeft.setPower(Range.clip(p*backLeft_displacement+i*areaBackLeft+d*speedBackLeft, min_negative, max_positive));
            backRight.setPower(Range.clip(p*backRight_displacement+i*areaBackRight+d*speedBackRight, min_negative, max_positive));

            frontLeft_displacement = frontLeft_encoder-frontLeft.getCurrentPosition();
            frontRight_displacement = frontRight_encoder-frontRight.getCurrentPosition();
            backLeft_displacement = backLeft_encoder-backLeft.getCurrentPosition();
            backRight_displacement = backRight_encoder-backRight.getCurrentPosition();
            telemetry.addData("area frontLeft", areaFrontLeft);
            telemetry.addData("area frontRight", areaFrontRight);
            telemetry.addData("area backLeft", areaBackLeft);
            telemetry.addData("area backRight", areaBackRight);
            telemetry.update();
            time.reset();
        }
        setPowerDriveMotors(0);
        //TODO i got rid of rotate
        //turnAbsolute(targetAngle);
    }
}
