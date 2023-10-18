// Simple autonomous program that drives bot forward until end of period
// or touch sensor is hit. If touched, backs up a bit and turns 90 degrees
// right and keeps going. Demonstrates obstacle avoidance and use of the
// REV Hub's built in IMU in place of a gyro. Also uses gamepad1 buttons to
// simulate touch sensor press and supports left as well as right turn.
//
// Also uses PID controller to drive in a straight line when not
// avoiding an obstacle.
//
// Use PID controller to manage motor power during 90 degree turn to reduce
// overshoot.

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name="Drive Avoid PID2", group="Exercises")
@Disabled
public class PID_Methods extends LinearOpMode {
    Orientation lastAngles = new Orientation();

    double                  globalAngle, power = 0.3, correction, rotation;
    boolean                 aButton, bButton, touched;

    PIDController pidDrive;
    PIDController pidRotate;

    Hardware r = new Hardware(this);

    PID_Methods() {
        pidDrive = new PIDController(.05, 0, 0);
        pidRotate = new PIDController(.003, .00003, 0);
    }

    // called when init button is  pressed.
    @Override
    public void runOpMode() throws InterruptedException {

        waitForStart();

    }


}