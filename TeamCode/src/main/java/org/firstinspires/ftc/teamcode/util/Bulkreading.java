package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.List;

public class Bulkreading {
    private LinearOpMode myOpMode;
    public Bulkreading(LinearOpMode opmode) {myOpMode = opmode;}
    public void setBulkCacheModeManual(List<LynxModule> allHubs) {
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }
    public void setBulkCacheModeAuto(List<LynxModule> allHubs) {
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
    }
    public void resetCache(List<LynxModule> allHubs) {
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }
    }
}
