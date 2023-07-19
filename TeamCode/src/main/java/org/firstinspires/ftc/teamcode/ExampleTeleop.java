package org.firstinspires.ftc.teamcode;

import android.content.SharedPreferences;

import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class ExampleTeleop extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        PhotonCore.enable();
        waitForStart();

        while (opModeIsActive()) {

        }
    }
}
