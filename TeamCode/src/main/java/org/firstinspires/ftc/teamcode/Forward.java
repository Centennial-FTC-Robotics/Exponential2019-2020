package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;
@TeleOp
public class Forward extends LinearOpMode {
    DcMotor backleft;
    DcMotor backright;
    DcMotor upleft;
    DcMotor upright;
    @Override
    public void runOpMode() throws InterruptedException {
        backleft=hardwareMap.dcMotor.get("lmotor0");
        backright=hardwareMap.dcMotor.get("rmotor0");
        upleft=hardwareMap.dcMotor.get("lmotor1");
        upright=hardwareMap.dcMotor.get("rmotor1");



        upright.setDirection(DcMotorSimple.Direction.REVERSE);
        backright.setDirection(DcMotorSimple.Direction.REVERSE);

        upleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        upright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        while(opModeIsActive()) {
            double power = gamepad1.left_stick_y;
            double rotate = gamepad1.right_stick_x;
            upleft.setPower(Range.clip(power + rotate, -1, 1));
            upright.setPower(Range.clip(power+rotate,-1,1));
            backleft.setPower(Range.clip(power-rotate,-1,1));
            backright.setPower(Range.clip(power+rotate,-1,1));


        }




    }




}
