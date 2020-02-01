package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

//import org.firstinspires.ftc.robotcontroller.external.samples.SampleRevBlinkinLedDriver;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

public abstract class Exponential_Hardware_Initializations extends LinearOpMode {

    protected DcMotor frontLeft;
    protected DcMotor frontRight;
    protected DcMotor backLeft;
    protected DcMotor backRight;
    protected DcMotor intakeLeft;
    protected DcMotor intakeRight;
    protected Servo intakeServoLeft;
    protected Servo intakeServoRight;
    protected Servo hookServoLeft;
    protected Servo hookServoRight;
    //protected Servo yeetServo;

    protected DcMotor odoWheelForwards;
    protected DcMotor odoWheelSideways;

    //protected RevBlinkinLedDriver blinkinLedDriver;
    protected RevBlinkinLedDriver blinkin;
    protected RevBlinkinLedDriver.BlinkinPattern pattern;
    //protected DisplayKind displayKind;

    protected DcMotor slideUp;
    protected DcMotor slideDown;
    protected DcMotor[] driveMotors = new DcMotor[4];
    protected ColorSensor sensorColor;
    protected DistanceSensor sensorDistance;

    protected Orientation orientation = new Orientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES,0,0,0,0);
    protected BNO055IMU imu;
    double initialHeading;
    double initialPitch;
    double initialRoll;

    @Override
    public void runOpMode() throws InterruptedException {
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        backRight = hardwareMap.dcMotor.get("backRight");
        intakeServoLeft = hardwareMap.servo.get("intakeServoLeft");
        intakeServoRight = hardwareMap.servo.get("intakeServoRight");
        //yeetServo = hardwareMap.servo.get("yeetServo");

        blinkin = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        pattern = RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE;
        blinkin.setPattern(pattern);

        hookServoLeft = hardwareMap.servo.get("hookServoLeft");
        hookServoRight = hardwareMap.servo.get("hookServoRight");

        intakeLeft = hardwareMap.dcMotor.get("intakeLeft");
        intakeRight = hardwareMap.dcMotor.get("intakeRight");
        slideUp = hardwareMap.dcMotor.get("slideRight");
        slideDown = hardwareMap.dcMotor.get("slideLeft");

        // odoWheelForwards = hardwareMap.dcMotor.get("OdometryWheelForwards");
        // odoWheelSideways = hardwareMap.dcMotor.get("OdometryWheelSideways");

        // sensorColor = hardwareMap.get(ColorSensor.class, "sensor_color_distance");
        // sensorDistance = hardwareMap.get(DistanceSensor.class, "sensor_color_distance");
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        intakeServoRight.setDirection(Servo.Direction.REVERSE);
        intakeRight.setDirection(DcMotorSimple.Direction.REVERSE);
        hookServoRight.setDirection(Servo.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        slideUp.setDirection(DcMotorSimple.Direction.REVERSE);
        // slideDown.setDirection(DcMotorSimple.Direction.REVERSE);

        driveMotors[0] = frontLeft;
        driveMotors[1] = frontRight;
        driveMotors[2] = backLeft;
        driveMotors[3] = backRight;

        for(DcMotor motor : driveMotors){
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        slideUp.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideDown.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideUp.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideDown.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideUp.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideDown.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
}