package org.firstinspires.ftc.teamcode.Testers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Hardware;

//@TeleOp(name = "Motor Tests", group = "TeleOp")
// @TeleOp(name = "Motor Tests", group = "TeleOp")
public class MotorTesting extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor test = hardwareMap.dcMotor.get("motor");
        waitForStart();
        while(opModeIsActive()){
            test.setPower(gamepad1.right_stick_x);
        }
    }
}
