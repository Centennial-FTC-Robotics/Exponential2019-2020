package org.firstinspires.ftc.teamcode.Trash;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Exponential_Methods;

@TeleOp(name = "Servo Tester")
public class servoTester extends Exponential_Methods {
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        double servoPos = 0;
        ElapsedTime time = new ElapsedTime();
        while(opModeIsActive()){
            if(gamepad1.right_stick_x!=0.0&&time.seconds()>.2){
                time.reset();
                servoPos+=.05*(int)(gamepad1.right_stick_x);
                servoPos = Range.clip(servoPos,0,1);
                hoodServo.setPosition(servoPos);
            }
            telemetry.addData("Servo Position", servoPos);
            telemetry.update();
        }
    }
}
