package org.firstinspires.ftc.teamcode.Trash;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Exponential_Methods;

import java.io.File;

@TeleOp(name = "Read file", group = "TeleOp")
public class readDisplacement extends Exponential_Methods {
    public void runOpMode() throws InterruptedException{
        super.runOpMode();
        File test = AppUtil.getInstance().getSettingsFile("test.txt");
        while(opModeIsActive()){
            telemetry.addData("data", ReadWriteFile.readFile(test));
            telemetry.update();
        }
    }
}
