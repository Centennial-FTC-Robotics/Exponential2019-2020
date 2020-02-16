package org.firstinspires.ftc.teamcode.Trash;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Exponential_Methods;

import java.io.File;

@TeleOp(name = "Read Write File")
public class ReadWriteFileTesting extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        File testCases = AppUtil.getInstance().getSettingsFile("ReadWriteFileFile.txt");
        String read = ReadWriteFile.readFile(testCases);
        while(opModeIsActive()){
            telemetry.addData("File", read);
            telemetry.update();
        }
    }
}
