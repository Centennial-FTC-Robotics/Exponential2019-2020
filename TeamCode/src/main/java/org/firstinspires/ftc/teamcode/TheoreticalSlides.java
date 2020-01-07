package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp(name = "Theoretical Slides", group = "TeleOp")
public class TheoreticalSlides extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor slideUp; DcMotor slideDown;
        slideUp = hardwareMap.dcMotor.get("slideRight");
        slideDown = hardwareMap.dcMotor.get("slideLeft");
        slideDown.setDirection(DcMotorSimple.Direction.REVERSE);
        slideDown.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideDown.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        waitForStart();
        int position = (slideUp.getCurrentPosition()+slideDown.getCurrentPosition())/2;
        while(opModeIsActive()){
            telemetry.addData("Y: ", gamepad2.left_stick_y);
            telemetry.update();
            if(gamepad2.left_stick_y!=0){
                position = (slideDown.getCurrentPosition()+slideUp.getCurrentPosition())/2;
                slideDown.setTargetPosition(position);
                slideUp.setTargetPosition(position);
                slideUp.setPower(gamepad2.left_stick_y);
                slideDown.setPower(gamepad2.left_stick_y);
            }
        }
    }
}
