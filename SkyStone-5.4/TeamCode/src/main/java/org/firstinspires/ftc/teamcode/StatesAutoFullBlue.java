package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name="StatesAutoFullBlue", group="Hardware Test")
@Disabled
public class StatesAutoFullBlue extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    Hardware r;

    Orientation lastAngles = new Orientation();

    double                  globalAngle, power = 0.3, correction, rotation;
    boolean                 aButton, bButton, touched;

    PIDController pidRotate = new PIDController(.009, .00005, 0);
    PIDController pidDrive = new PIDController(.005, 0, 0);

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        r = new Hardware(this);

        r.startVision("Blue");

        waitForStart();
        runtime.reset();

        r.setLEDs(RevBlinkinLedDriver.BlinkinPattern.SINELON_FOREST_PALETTE);


        //r.TankTurn(0.8,780);

        //rotateWithArm(90, 0.3, -0.75, 7);
        //Thread.sleep(100000);

        int stone = r.SkyStoneLocation();
        telemetry.addData("stone: ", stone);
        telemetry.update();

        //int stone = 2;

        if (stone == 3){
            r.MecanaumStrafe(0.75,-850);
        } else if (stone == 2) {
            r.MecanaumStrafe(0.75,-300);
        } else if (stone == 1){
            //Already here
        }

        /*
        r.servo4.setPosition(0.6); //Grabber out
        r.servo3.setPosition(0.7);
        Thread.sleep(750);
        r.ArmPower(1); //Arm Up
        r.servo4.setPosition(0.09); //Close Grabber

         */




        rotate(-90, 0.3); //Rotate to Foundation
        r.TankForward(.25, 555); //Drive to Foundation wsa 525


        r.foundationMovePower(-1); //Grab Foundation
        Thread.sleep(500);
        r.TankForward(0.8, -1500); //Move forward with foundation
        //rotate(-90, 0.6); //Turn foundation

        r.ArmPower(-1);
        r.servo4.setPosition(.6);
        Thread.sleep(200);
        r.servo4.setPosition(.09);
        Thread.sleep(1000);

        //rotateWithArm(-90, 0.8, 1.0, 4);
        TankTurnWithArm(0.3, -2200, 1.0, 4);

        /*
        //Makes sure  it has block
        r.servo4.setPosition(0.4);
        r.ArmPower(-1);
        Thread.sleep(500);
        r.servo4.setPosition(0.09);
        Thread.sleep(400);

         */

        runtime.reset();
        /*
        while (runtime.milliseconds() < 2500) {
            r.ArmPower(1); //Arm Up
            if ((int) runtime.milliseconds()%4 == 0) {
                r.ArmPower(0);
                Thread.sleep(3);
            }
            r.servo3.setPosition(Math.sin(r.ArmAngle.getVoltage()/2.45) + 0.029); //Angle of Grab
        }

         */
        r.IntakePower(0); //Intake Off
        r.servo4.setPosition(0.5); //Open Grabber
        r.ArmPower(-1); //Arm In
        Thread.sleep(200);

        r.foundationMovePower(1); //Let go of Foundation
        r.TankForward(.55, 900); //Go agaisnt wall

        r.MecanaumStrafe(0.75,240);


        r.TankForward(0.6, -1400); //Drive to park
        r.ArmPower(0); //Arm off


    }

    /**
     * Resets the cumulative angle tracking to zero.
     */
    private void resetAngle()
    {
        lastAngles = r.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right from zero point.
     */
    private double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = r.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    /**
     * Rotate left or right the number of degrees. Does not support turning more than 359 degrees.
     * @param degrees Degrees to turn, + is left - is right
     */
    public void rotate(int degrees, double power) throws InterruptedException {
        // restart imu angle tracking.
        r.LTMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        r.LBMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        r.RTMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        r.RBMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        resetAngle();

        // if degrees > 359 we cap at 359 with same sign as original degrees.
        if (Math.abs(degrees) > 359) degrees = (int) Math.copySign(359, degrees);

        // start pid controller. PID controller will monitor the turn angle with respect to the
        // target angle and reduce power as we approach the target angle. This is to prevent the
        // robots momentum from overshooting the turn after we turn off the power. The PID controller
        // reports onTarget() = true when the difference between turn angle and target angle is within
        // 1% of target (tolerance) which is about 1 degree. This helps prevent overshoot. Overshoot is
        // dependant on the motor and gearing configuration, starting power, weight of the robot and the
        // on target tolerance. If the controller overshoots, it will reverse the sign of the output
        // turning the robot back toward the setpoint value.

        pidRotate.reset();
        pidRotate.setSetpoint(degrees);
        pidRotate.setInputRange(0, degrees);
        pidRotate.setOutputRange(0, power);
        pidRotate.setTolerance(1);
        pidRotate.enable();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        // rotate until turn is completed.

        if (degrees < 0)
        {
            // On right turn we have to get off zero first.

            while (opModeIsActive() && getAngle() == 0)
            {
                r.LTMotor.setPower(power);
                r.LBMotor.setPower(power);
                r.RTMotor.setPower(-power);
                r.RBMotor.setPower(-power);
                sleep(100);
            }



            do
            {
                power = pidRotate.performPID(getAngle()); // power will be - on right turn.
                r.LTMotor.setPower(-power);
                r.LBMotor.setPower(-power);
                r.RTMotor.setPower(power);
                r.RBMotor.setPower(power);
            } while (opModeIsActive() && !pidRotate.onTarget());
        }
        else    // left turn.
            do
            {
                power = pidRotate.performPID(getAngle()); // power will be + on left turn.
                r.LTMotor.setPower(-power);
                r.LBMotor.setPower(-power);
                r.RTMotor.setPower(power);
                r.RBMotor.setPower(power);
            } while (opModeIsActive() && !pidRotate.onTarget());

        // turn the motors off.
        r.RobotStop();

        rotation = getAngle();

        // wait for rotation to stop.
        sleep(500);

        // reset angle tracking on new heading.
        resetAngle();
    }

    public void TankForwardStraight(double power, int target){
        r.LTMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        r.LBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        r.RTMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        r.RBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        if (target != 0) {

            pidDrive.setSetpoint(0);
            pidDrive.setOutputRange(0, power);
            pidDrive.setInputRange(-90, 90);
            pidDrive.enable();

            r.RBMotor.setTargetPosition(target);
            r.RTMotor.setTargetPosition(target);
            r.LBMotor.setTargetPosition(target);
            r.LTMotor.setTargetPosition(target);

            r.LTMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            r.LBMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            r.RTMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            r.RBMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while (r.LTMotor.isBusy() && r.RTMotor.isBusy()) {

                correction = pidDrive.performPID(getAngle());

                telemetry.addData("1 imu heading", lastAngles.firstAngle);
                telemetry.addData("2 global heading", globalAngle);
                telemetry.addData("3 correction", correction);
                telemetry.addData("4 turn rotation", rotation);
                telemetry.update();

                r.OpenPower(power + correction, power + correction, power - correction, power - correction);
            }
            r.RobotStop();
        } else {

            r.LTMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            r.LBMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            r.RTMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            r.RBMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            r.RTMotor.setPower(power);
            r.RBMotor.setPower(power);
            r.LTMotor.setPower(power);
            r.LBMotor.setPower(power);
        }
    }

    public void rotateWithArm(int degrees, double power, double ArmPower, int modulus) throws InterruptedException {
        // restart imu angle tracking.
        r.LTMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        r.LBMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        r.RTMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        r.RBMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        resetAngle();

        // if degrees > 359 we cap at 359 with same sign as original degrees.
        if (Math.abs(degrees) > 359) degrees = (int) Math.copySign(359, degrees);

        // start pid controller. PID controller will monitor the turn angle with respect to the
        // target angle and reduce power as we approach the target angle. This is to prevent the
        // robots momentum from overshooting the turn after we turn off the power. The PID controller
        // reports onTarget() = true when the difference between turn angle and target angle is within
        // 1% of target (tolerance) which is about 1 degree. This helps prevent overshoot. Overshoot is
        // dependant on the motor and gearing configuration, starting power, weight of the robot and the
        // on target tolerance. If the controller overshoots, it will reverse the sign of the output
        // turning the robot back toward the setpoint value.

        pidRotate.reset();
        pidRotate.setSetpoint(degrees);
        pidRotate.setInputRange(0, degrees);
        pidRotate.setOutputRange(0, power);
        pidRotate.setTolerance(1);
        pidRotate.enable();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        // rotate until turn is completed.

        if (degrees < 0)
        {
            // On right turn we have to get off zero first.

            runtime.reset();
            while (opModeIsActive() && getAngle() == 0)
            {
                r.LTMotor.setPower(power);
                r.LBMotor.setPower(power);
                r.RTMotor.setPower(-power);
                r.RBMotor.setPower(-power);
                sleep(100);


                r.ArmPower(ArmPower); //Arm Down
                if ((int) runtime.milliseconds()%modulus == 0) {
                    r.ArmPower(0);
                    Thread.sleep(3);
                }
                r.servo3.setPosition(Math.sin(r.ArmAngle.getVoltage()/2.45) + 0.029); //Angle of Grab
            }


            runtime.reset();
            do
            {
                power = pidRotate.performPID(getAngle()); // power will be - on right turn.
                r.LTMotor.setPower(-power);
                r.LBMotor.setPower(-power);
                r.RTMotor.setPower(power);
                r.RBMotor.setPower(power);

                r.ArmPower(ArmPower); //Arm Down
                if ((int) runtime.milliseconds()%modulus == 0) {
                    r.ArmPower(0);
                    Thread.sleep(3);
                }
                r.servo3.setPosition(Math.sin(r.ArmAngle.getVoltage()/2.45) + 0.029); //Angle of Grab

            } while (opModeIsActive() && !pidRotate.onTarget());
        }
        else    // left turn.
            runtime.reset();
            do
            {
                power = pidRotate.performPID(getAngle()); // power will be + on left turn.
                r.LTMotor.setPower(-power);
                r.LBMotor.setPower(-power);
                r.RTMotor.setPower(power);
                r.RBMotor.setPower(power);

                r.ArmPower(ArmPower); //Arm Down
                if ((int) runtime.milliseconds()%modulus == 0) {
                    r.ArmPower(0);
                    Thread.sleep(3);
                }
                r.servo3.setPosition(Math.sin(r.ArmAngle.getVoltage()/2.45) + 0.029); //Angle of Grab

            } while (opModeIsActive() && !pidRotate.onTarget());

        // turn the motors off.
        r.RobotStop();

        rotation = getAngle();

        // wait for rotation to stop.
        sleep(500);

        // reset angle tracking on new heading.
        resetAngle();
    }

    public void TankTurnWithArm(double power, int target, double ArmPower, int modulus) throws InterruptedException {
        r.LTMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        r.LBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        r.RTMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        r.RBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        if (target != 0) {

            r.RBMotor.setTargetPosition(-target);
            r.RTMotor.setTargetPosition(-target);
            r.LBMotor.setTargetPosition(target);
            r.LTMotor.setTargetPosition(target);

            r.LTMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            r.LBMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            r.RTMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            r.RBMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while (r.LTMotor.isBusy() && r.RTMotor.isBusy()) {
                r.OpenPower(power, power, power, power);

                r.ArmPower(ArmPower); //Arm Down
                if ((int) runtime.milliseconds()%modulus == 0) {
                    r.ArmPower(0);
                    Thread.sleep(3);
                }
                r.servo3.setPosition(Math.sin(r.ArmAngle.getVoltage()/2.45) + 0.029); //Angle of Grab
            }
            r.RobotStop();
        } else {

            r.LTMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            r.LBMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            r.RTMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            r.RBMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            r.RTMotor.setPower(-power);
            r.RBMotor.setPower(-power);
            r.LTMotor.setPower(power);
            r.LBMotor.setPower(power);
        }
    }
}
