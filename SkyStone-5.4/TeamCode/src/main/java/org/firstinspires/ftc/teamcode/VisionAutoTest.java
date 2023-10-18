package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Vision auto", group="Hardware Test")
public class VisionAutoTest extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        Hardware r = new Hardware(this);

        r.startVision("Blue");

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            telemetry.addData("Location: ", r.SkyStoneLocation());
            telemetry.update();

        }

        Thread.sleep(5000);
    }
}
