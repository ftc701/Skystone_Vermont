package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;

@TeleOp(name="States", group="Hardware Test")
@Disabled
public class StatesTeleOp extends LinearOpMode {

    float mod = 1;

    boolean lockLift = false;

    private final static int GAMEPAD_LOCKOUT = 500;
    Deadline gamepadRateLimit;
    Hardware r;

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime time = new ElapsedTime();
    private ElapsedTime LEDtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        r = new Hardware(this);

        gamepadRateLimit = new Deadline(GAMEPAD_LOCKOUT, TimeUnit.MILLISECONDS);

        waitForStart();
        runtime.reset();
        LEDtime.reset();

        while (opModeIsActive()) {

            //r.servo1.setPower(-gamepad2.left_stick_y);
            //r.servo2.setPower(-gamepad2.left_stick_y);
            //r.servo3.setPosition(Math.sin(r.ArmAngle.getVoltage()/2.45) + 0.029);
            double xVal = r.ArmAngle.getVoltage();
            //r.servo3.setPosition((0.824097/(1+1.8275 * Math.pow(10,8) * Math.pow(Math.E,-14.9923 * xVal)))+0.135);
            double[] coord1 = {0.263, 0.135};
            double[] coord2 = {2.168, 0.8};
            //r.servo3.setPosition(r.mapFunction(xVal,coord1,coord2));
            r.servo3.setPosition((0.680135/(1+82.009 * Math.pow(Math.E,-3.47386 * xVal)))+0.135);

            if (gamepad2.left_bumper) {
                r.servo4.setPosition(0.55);
            } else if (gamepad2.right_bumper) {
                r.servo4.setPosition(0.09);
            }

            LockLiftLogic();

            handleSpeed();

            float LTMotorDC = gamepad1.left_stick_y - gamepad1.left_stick_x +  (0.5f) * gamepad1.right_stick_x;
            float LBMotorDC = gamepad1.left_stick_y + gamepad1.left_stick_x + (0.5f) * gamepad1.right_stick_x;
            float RTMotorDC = gamepad1.left_stick_y + gamepad1.left_stick_x - (0.5f) * gamepad1.right_stick_x;
            float RBMotorDC = gamepad1.left_stick_y - gamepad1.left_stick_x - (0.5f) * gamepad1.right_stick_x;

            float largestVal = 1.0f;
            largestVal = Math.max(largestVal, Math.abs(LTMotorDC));
            largestVal = Math.max(largestVal, Math.abs(LBMotorDC));
            largestVal = Math.max(largestVal, Math.abs(RTMotorDC));
            largestVal = Math.max(largestVal, Math.abs(RBMotorDC));

            LTMotorDC = LTMotorDC/largestVal;
            LBMotorDC = LBMotorDC/largestVal;
            RTMotorDC = RTMotorDC/largestVal;
            RBMotorDC = RBMotorDC/largestVal;

            LTMotorDC = LTMotorDC * mod;
            LBMotorDC = LBMotorDC * mod;
            RTMotorDC = RTMotorDC * mod;
            RBMotorDC = RBMotorDC * mod;

            r.OpenPower(LTMotorDC, LBMotorDC, RTMotorDC, RBMotorDC);

            r.intake1.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
            r.intake2.setPower(gamepad1.right_trigger - gamepad1.left_trigger);

            r.lift1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            r.lift1.setPower((gamepad2.left_trigger - gamepad2.right_trigger));

            if (gamepad1.right_bumper){
                r.foundationMovePower(-1);
            } else if (gamepad1.left_bumper) {
                r.foundationMovePower(1);
            }

            if (gamepad2.dpad_left) {
                r.servo7.setPosition(0.85);
            } else if (gamepad2.dpad_right){
                r.servo7.setPosition(0.45);
            }

            ledControl();

            r.showTelemtry(this);

        }
    }

    public void handleSpeed(){
        if (!gamepadRateLimit.hasExpired()) {
            return;
        }

        if (gamepad1.x && mod == 0.4f){
            mod = 1.0f;
            gamepadRateLimit.reset();
        } else if (gamepad1.x && mod == 1.0f){
            mod = 0.4f;
            gamepadRateLimit.reset();
        }
    }

    public void LockLiftLogic(){
        if (r.lift1.getPower() != 0 && gamepad2.a){
            lockLift = false;
        }

        if (gamepad2.a){
            lockLift = true;
        }

        if (r.lift1.getPower() == 0 && lockLift){
            r.lift1.setPower(-0.125);
        }
    }

    public void ledControl(){
        if (LEDtime.seconds() > 60 && LEDtime.seconds() < 90){
            r.setLEDs(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
        } else if (LEDtime.seconds() > 90 && LEDtime.seconds() < 105){
            r.setLEDs(RevBlinkinLedDriver.BlinkinPattern.ORANGE);
        } else if (LEDtime.seconds() > 105){
            r.setLEDs(RevBlinkinLedDriver.BlinkinPattern.STROBE_RED);
        } else {
            r.setLEDs(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE);
        }
    }
}
