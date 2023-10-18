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

@Autonomous(name="VTAutoRedFoundation", group="VT Auto")
public class VTAutoRedFoundation extends LinearOpMode {

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
            if (gamepad1.left_bumper){
                parkLocation = 1;
            } else if (gamepad1.right_bumper){
                parkLocation = 2;
            }

            handleWaitTimer();


            telemetry.addData("Park Location: ", parkLocation);
            telemetry.addData("WaitTime: ", waitTime);
            telemetry.update();
        }

        runtime.reset();

        Thread.sleep(waitTime);

        r.TankForward(0.25, 1300); //Go to foundation
        r.foundationMovePower(-1); //put down foundation grabbers
        Thread.sleep(1000);

        r.TankForward(0.6, -975); //go back to depot
        r.TankTurn(0.76,-1800); //turn to put foundation grabber in depot
        r.foundationMovePower(1); //let go of foundation
        Thread.sleep(1000);
        r.TankForward(.55, 900); //back agasint wall


        if (parkLocation == 1){
            r.MecanaumStrafe(0.4, -500);
        } else if (parkLocation == 2){
            r.MecanaumStrafe(0.4, 400);
        }

        r.TankForward(0.6, -1400); //move to park


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


