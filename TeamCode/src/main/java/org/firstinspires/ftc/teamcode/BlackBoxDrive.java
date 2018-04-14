package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name = "EncoderDrive")
public class BlackBoxDrive extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private double righty = 0;
    private double rightx = 0;
    private double leftx = 0;
    private double lefty = 0;
    static final double threshold = 0.3;
    private BlackBoxBot bb = new BlackBoxBot();
    public void runOpMode() throws InterruptedException {
        bb.init(hardwareMap);
        telemetry.update();
        runtime.reset();
        waitForStart();
        while (!isStopRequested() && opModeIsActive()) {
            lefty = Range.clip(gamepad1.left_stick_y, -1, 1);
            righty = Range.clip(gamepad1.right_stick_y, -1, 1);
            rightx = Range.clip(gamepad1.right_stick_x, -1, 1);
            leftx = Range.clip(gamepad1.left_stick_x, -1, 1);
            if (Math.abs(gamepad1.left_stick_y) > threshold) {
                bb.drive(-lefty, -lefty, -lefty, -lefty);
            } else if (Math.abs(gamepad1.left_stick_x) > threshold) {
                bb.drive(-leftx, leftx, leftx, -leftx);
            }  else if (Math.abs(gamepad1.right_stick_x) > threshold) {
                bb.drive(-rightx, -rightx, rightx, rightx);
            } else {
                bb.drive(0, 0, 0, 0);
            }
            idle();
        }
        bb.drive(0, 0, 0, 0);
    }
    public double map(double x, double in_min, double in_max, double out_min, double out_max) {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }
}
