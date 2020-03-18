package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
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

    protected DcMotorEx frontLeft;
    protected DcMotorEx frontRight;
    protected DcMotorEx backLeft;
    protected DcMotorEx backRight;
    protected DcMotorEx intakeLeft;
    protected DcMotorEx intakeRight;
    protected Servo intakeServoLeft;
    protected Servo intakeServoRight;
    protected Servo hookServoLeft;
    protected Servo hookServoRight;
    protected Servo yeetServo;
    protected Servo hoodServo;
    protected Servo claw;

    protected DcMotor odoWheelForwards;
    protected DcMotor odoWheelSideways;

    //protected RevBlinkinLedDriver blinkinLedDriver;
    protected RevBlinkinLedDriver blinkin;
    protected RevBlinkinLedDriver.BlinkinPattern pattern;
    //protected DisplayKind displayKind;

    protected DcMotorEx slideUp;
    protected DcMotorEx slideDown;
    protected DcMotor[] driveMotors = new DcMotor[4];
    protected ColorSensor sensorColor;
    protected DistanceSensor sensorDistance;

    protected Orientation orientation = new Orientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES,0,0,0,0);
    protected BNO055IMU imu;
    public double initialHeading;
    public double initialPitch;
    public double initialRoll;

    @Override
    public void runOpMode() throws InterruptedException {
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        intakeServoLeft = hardwareMap.servo.get("intakeServoLeft");
        intakeServoRight = hardwareMap.servo.get("intakeServoRight");
        yeetServo = hardwareMap.servo.get("yeetServo");
        claw = hardwareMap.servo.get("claw");
        // hoodServo = hardwareMap.servo.get("hoodServo");


        blinkin = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        pattern = RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE;
        blinkin.setPattern(pattern);

        hookServoLeft = hardwareMap.servo.get("hookServoLeft");
        hookServoRight = hardwareMap.servo.get("hookServoRight");

        intakeLeft = hardwareMap.get(DcMotorEx.class, "intakeLeft");
        intakeRight = hardwareMap.get(DcMotorEx.class, "intakeRight");
        slideUp = hardwareMap.get(DcMotorEx.class, "slideRight");
        slideDown = hardwareMap.get(DcMotorEx.class, "slideLeft");

        odoWheelForwards = intakeLeft;
        //Encoder cable for forwards odometry wheel is plugged into the intakeLeft motor encoder
        odoWheelSideways = intakeRight;

        // sensorColor = hardwareMap.get(ColorSensor.class, "sensor_color_distance");
        // sensorDistance = hardwareMap.get(DistanceSensor.class, "sensor_color_distance");
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        intakeServoRight.setDirection(Servo.Direction.REVERSE);
        intakeRight.setDirection(DcMotorEx.Direction.REVERSE);
        hookServoRight.setDirection(Servo.Direction.REVERSE);
        frontRight.setDirection(DcMotorEx.Direction.REVERSE);
        backRight.setDirection(DcMotorEx.Direction.REVERSE);
        slideUp.setDirection(DcMotorEx.Direction.REVERSE);
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

        odoWheelSideways.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        odoWheelForwards.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        odoWheelSideways.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        odoWheelForwards.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        slideUp.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slideDown.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slideUp.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        slideDown.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        slideUp.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        slideDown.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }
}