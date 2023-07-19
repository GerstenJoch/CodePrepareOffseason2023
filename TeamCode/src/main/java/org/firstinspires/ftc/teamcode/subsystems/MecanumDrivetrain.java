package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Config
public class MecanumDrivetrain {
    private LinearOpMode myOpMode;

    private DcMotor FrontL;
    private DcMotor FrontR;
    private DcMotor BackL;
    private DcMotor BackR;

    BNO055IMU imu;
    Orientation anglesHead;

    public static double cmPerTickY = 0.0028;
    public static double cmPerTickX = 0.0018;

    private DcMotor leftEncoder, middleEncoder, rightEncoder;
    private double currentHeading;

    public MecanumDrivetrain(LinearOpMode opmode) {myOpMode = opmode;}
    public void init() {
        FrontL = myOpMode.hardwareMap.get(DcMotor.class, "leftFront");
        FrontR = myOpMode.hardwareMap.get(DcMotor.class, "rightFront");
        BackL = myOpMode.hardwareMap.get(DcMotor.class, "leftRear");
        BackR = myOpMode.hardwareMap.get(DcMotor.class, "rightRear");

        FrontL.setDirection(DcMotorSimple.Direction.REVERSE);
        FrontR.setDirection(DcMotorSimple.Direction.REVERSE);

        FrontL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FrontR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftEncoder = myOpMode.hardwareMap.dcMotor.get("leftFront");
        rightEncoder = myOpMode.hardwareMap.dcMotor.get("leftRear");
        middleEncoder = myOpMode.hardwareMap.dcMotor.get("rightFront");

        leftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        middleEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        resetIMU();
        new Thread(() -> getCurrentHeading()).start();
    }
    public void FieldCentric(double speed) {
        double theta = currentHeading + (3.1415 / 2);
        double FWD = (myOpMode.gamepad1.left_stick_x * Math.sin(theta) + myOpMode.gamepad1.left_stick_y * Math.cos(theta));
        double STR = (myOpMode.gamepad1.left_stick_x * Math.cos(theta) - myOpMode.gamepad1.left_stick_y * Math.sin(theta));
        double ROT = myOpMode.gamepad1.right_stick_x;
        speed = speed * -1;

        FrontL.setPower((FWD + STR + ROT) * (speed));
        FrontR.setPower((FWD - STR + ROT) * (speed));
        BackL.setPower((FWD - STR - ROT) * (speed));
        BackR.setPower((FWD + STR - ROT) * (speed));

        if (myOpMode.gamepad1.right_trigger > 0 && myOpMode.gamepad1.left_trigger > 0) {
            resetIMU();
        }
    }
    public void RobotCentric(double speed) {
        double FWD = myOpMode.gamepad1.left_stick_y;
        double STR = myOpMode.gamepad1.left_stick_x;
        double ROT = myOpMode.gamepad1.right_stick_x;
        speed = speed * -1;

        FrontL.setPower((FWD + STR + ROT) * (speed));
        FrontR.setPower((FWD - STR + ROT) * (speed));
        BackL.setPower((FWD - STR - ROT) * (speed));
        BackR.setPower((FWD + STR - ROT) * (speed));
    }
    //Autonomous drive in any direction
    public void Drive(double target_x, double target_y, double speed) {
        double Kp = 0.03;
        double turn;
        double heading = currentHeading + (3.1415 / 2);
        double cur_x = middleEncoder.getCurrentPosition() * cmPerTickX;
        double cur_y = (leftEncoder.getCurrentPosition() + rightEncoder.getCurrentPosition()) * 0.5 * cmPerTickY;
        double Vy, Vx, FWD, STR;
        while (myOpMode.opModeIsActive() && (target_x - cur_x > -0.05 && target_x - cur_x < 0.05) && (target_y - cur_y > -0.05 && target_y - cur_y < 0.05)) {
            Vy = ((target_x-cur_x) * Math.sin(heading) + (target_y-cur_y) * Math.cos(heading));
            Vx = ((target_x-cur_x) * Math.cos(heading) - (target_y-cur_y) * Math.sin(heading));
            if (!(target_y - cur_y > -0.05 && target_y - cur_y < 0.05))
                FWD = speed / (Vy + Vx) * Vy * checkDirection(target_y - cur_y);
            else FWD = 0;
            if (!(target_x - cur_x > -0.05 && target_x - cur_x < 0.05))
                STR = speed / (Vy + Vx) * Vx * checkDirection(target_x - cur_x);
            else STR = 0;

            turn = Kp*Math.abs(speed)*(getTargetHeading(heading)-currentHeading);

            FrontL.setPower(FWD + STR + turn);
            FrontR.setPower(FWD - STR - turn);
            BackL.setPower(FWD - STR + turn);
            BackR.setPower(FWD + STR - turn);

            cur_x = middleEncoder.getCurrentPosition() * cmPerTickX;
            cur_y = (leftEncoder.getCurrentPosition() + rightEncoder.getCurrentPosition()) * 0.5 * cmPerTickY;
        }
    }
    int checkDirection(double val){
        if (val < 0)
            return -1;
        else return 1;
    }

    public void getCurrentHeading() { //Threaded
        while (myOpMode.opModeIsActive()){
            anglesHead   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            currentHeading = getTargetHeading((int)(-1*anglesHead.firstAngle));
        }
    }
    public double getTargetHeading(double heading) {
        if(heading>181&&heading<360){
            return heading-360;
        }
        else return heading;
    }
    public void resetIMU() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        imu = myOpMode.hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }
}

