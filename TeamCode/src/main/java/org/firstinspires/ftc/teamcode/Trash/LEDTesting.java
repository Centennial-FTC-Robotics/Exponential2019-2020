package org.firstinspires.ftc.teamcode.Trash;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Exponential_Methods;

@Autonomous(group = "Autonomous", name = "LEDTesting")
public class LEDTesting extends Exponential_Methods {
    public void runOpMode() throws InterruptedException { // starts on second tile from the right
        super.runOpMode();
        waitForStart();
        sleep(1000);

        while(opModeIsActive()) {
            pattern = RevBlinkinLedDriver.BlinkinPattern.RED;
            blinkin.setPattern(pattern);
            telemetry.addData("doing red", "");
            telemetry.update();
            sleep(1000);
            pattern = RevBlinkinLedDriver.BlinkinPattern.AQUA;
            blinkin.setPattern(pattern);
            telemetry.addData("doing aqua", "");
            telemetry.update();
            sleep(1000);

        }
    }
}
