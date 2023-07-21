package org.firstinspires.ftc.teamcode;

import android.content.SharedPreferences;

import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.Bulkreading;

import java.util.List;

@TeleOp
public class ExampleTeleop extends LinearOpMode {
    private List<LynxModule> allhubs = hardwareMap.getAll(LynxModule.class);
    private Bulkreading hubs = new Bulkreading(this);

    @Override
    public void runOpMode() throws InterruptedException {
        PhotonCore.enable();
        hubs.setBulkCacheModeManual(allhubs);
        waitForStart();

        while (opModeIsActive()) {
            hubs.resetCache(allhubs);
        }
    }
}
