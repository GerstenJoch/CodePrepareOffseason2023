package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
public class Lift {
    private LinearOpMode myOpMode;
    private PIDController controller;

    public static double p = 0, i = 0, d =0;
    public static double f = 0;

    public static int target = 0;

    private final double ticks_in_degree = 537.7 / 180;

    private DcMotorEx lift_motor;
    public Lift(LinearOpMode opmode) {myOpMode = opmode;}
    public void init() {
        controller = new PIDController(p,i,d);
        myOpMode.telemetry = new MultipleTelemetry(myOpMode.telemetry, FtcDashboard.getInstance().getTelemetry());

        lift_motor = myOpMode.hardwareMap.get(DcMotorEx.class, "lift");

    }
    public void runToPosition(/*int target*/) {
        controller.setPID(p,i,d);
        int liftPos = lift_motor.getCurrentPosition();
        double pid = controller.calculate(liftPos, target);
        double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) *f;

        double power = pid + ff;

        lift_motor.setPower(power);

        myOpMode.telemetry.addData("pos: ", liftPos);
        myOpMode.telemetry.addData("target: ", target);
    }
}
