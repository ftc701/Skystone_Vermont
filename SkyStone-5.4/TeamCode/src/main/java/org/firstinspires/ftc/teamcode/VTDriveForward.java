package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;

@Autonomous(name="VTDriveForward", group="VT Auto")
public class VTDriveForward extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    Hardware r;

    Orientation lastAngles = new Orientation();

    double                  globalAngle, power = 0.3, correction, rotation;
    boolean                 aButton, bButton, touched;

    PIDController pidRotate = new PIDController(.009, .00005, 0);
    PIDController pidDrive = new PIDController(.005, 0, 0);

    int parkLocation = 2;
    int waitTime = 0;

    private final static int GAMEPAD_LOCKOUT = 150;
    Deadline gamepadRateLimit;

    @Override
    public void runOpMode() throws InterruptedException {

        r = new Hardware(this);

        gamepadRateLimit = new Deadline(GAMEPAD_LOCKOUT, TimeUnit.MILLISECONDS);

        while(!opModeIsActive() && !isStopRequested()) {

            handleWaitTimer();

            telemetry.addData("Place Robot with Intake Wheels Toward Parking Line", "");
            telemetry.addData("WaitTime: ", waitTime);
            telemetry.update();
        }

        runtime.reset();

        Thread.sleep(waitTime);

        r.TankForward(0.8, -800);

    }


    public void handleWaitTimer(){
        if (!gamepadRateLimit.hasExpired()) {
            return;
        }

        if (gamepad1.a){
            waitTime = waitTime + 500;
            gamepadRateLimit.reset();
        } else if (gamepad1.b && waitTime > 0){
            waitTime = waitTime - 500;
            gamepadRateLimit.reset();
        }
    }

}


