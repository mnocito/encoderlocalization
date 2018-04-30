package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Print POsItiOn")
public class BlackBoxPrint extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private BlackBoxBot bb = new BlackBoxBot();
    public void runOpMode() throws InterruptedException {
        bb.init(hardwareMap);
        runtime.reset();
        bb.resetTicks();
        telemetry.addData("status", "initialized");
        telemetry.update();
        waitForStart();
        while (!isStopRequested() && opModeIsActive()) {
            bb.updatePosition();
            telemetry.addData("Left ticks", bb.getLeftTicks());
            telemetry.addData("Center ticks", bb.getCenterTicks());
            telemetry.addData("Right ticks", bb.getRightTicks());
            telemetry.addData("X value", bb.getX());
            telemetry.addData("Y value", bb.getY());
            telemetry.addData("Theta value", Math.toDegrees(bb.getTheta()));
            telemetry.update();
            idle();
        }
    }
}
