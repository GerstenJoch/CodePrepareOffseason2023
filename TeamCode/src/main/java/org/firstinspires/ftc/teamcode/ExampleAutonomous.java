package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.util.Bulkreading;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;

import java.util.List;

@Config
@Autonomous
public class ExampleAutonomous extends LinearOpMode {
    private List<LynxModule> allhubs = hardwareMap.getAll(LynxModule.class);
    private Bulkreading hubs = new Bulkreading(this);

    @Override
    public void runOpMode() {
        PhotonCore.enable();
        hubs.setBulkCacheModeManual(allhubs);
        waitForStart();

        while (opModeIsActive()) {
            hubs.resetCache(allhubs);
        }
    }
}