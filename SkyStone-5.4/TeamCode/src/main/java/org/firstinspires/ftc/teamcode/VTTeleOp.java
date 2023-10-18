package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;

@TeleOp(name="VTStates", group="VT Auto")
public class VTTeleOp extends LinearOpMode {

    float mod = 1;

    boolean lockLift = false;
    boolean runOnce = true;

    private final static int GAMEPAD_LOCKOUT = 500;
    Deadline gamepadRateLimit;
    Hardware r;

    boolean disengadeLift;

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime time = new ElapsedTime();
    private ElapsedTime LEDtime = new ElapsedTime();
    private ElapsedTime liftTime = new ElapsedTime();

    // Declare uh oh stinky
    boolean liftTimerLockOut = false;

    boolean armGoingDown = true;

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

            if (runOnce){
                r.ArmPosition(0.82);
                runOnce = false;
            }

            double delPosition = (gamepad2.left_stick_y * 0.03);
            if (delPosition > 0) {
                armGoingDown = true;
            } else if (delPosition < 0){
                armGoingDown = false;
            }
            double ArmPositon = (gamepad2.left_stick_y * 0.03) + r.servo1.getPosition();
            ArmPositon = Range.clip(ArmPositon, 0,1);

            r.servo1.setPosition(ArmPositon);
            r.servo2.setPosition(ArmPositon);


            if (gamepad2.dpad_up && runtime.milliseconds() > 240) {
                r.servo3.setPosition(0.03 + r.servo3.getPosition());
                runtime.reset();
            } else if (gamepad2.dpad_down && runtime.milliseconds() > 240) {
                r.servo3.setPosition(-0.03 + r.servo3.getPosition());
                runtime.reset();
            }

            if (gamepad2.right_stick_button){
                r.servo3.setPosition((0.680135/(1+82.009 * Math.pow(Math.E,-3.47386 * r.ArmAngle.getVoltage())))+0.135);
            }


            //r.servo3.setPosition(Math.sin(r.ArmAngle.getVoltage()/2.45) + 0.029);
            double xVal = r.ArmAngle.getVoltage();
            //double armPosition = (1.1861/(1+0.259626 * Math.pow(Math.E,1.75895 * xVal)))+0.029;
            double armPosition = 0.0586966 * Math.pow(xVal, 2) + 0.147808 * xVal - 0.166477;
            if (armGoingDown) {
                armPosition = 0.0586966 * Math.pow(xVal, 2) + 0.147808 * xVal - 0.166477;
                //r.servo3.setPosition((0.824097/(1+1.8275 * Math.pow(10,8) * Math.pow(Math.E,-14.9923 * xVal)))+0.135);
                //double[] coord1 = {0.263, 0.135};
                //double[] coord2 = {2.168, 0.8};
                //r.servo3.setPosition(r.mapFunction(xVal,coord1,coord2));
                //r.servo3.setPosition((0.680135/(1+82.009 * Math.pow(Math.E,-3.47386 * xVal)))+0.135);
                if (xVal > 2.0 && xVal < 3.2) {
                    armPosition = 0.78;
                } else if (xVal > 3.2) {
                    double[] coord3 = {3.2, 0.78};
                    double[] coord4 = {3.25, 0.6};
                    armPosition = r.mapFunction(xVal, coord3, coord4);
                }
            }  else if (!armGoingDown) {
                armPosition = 0.0586966 * Math.pow(xVal, 2) + 0.147808 * xVal - 0.166477;
                if (xVal > 2.693){
                    double[] coord3 = {2.693,0.6616};
                    double[] coord4 = {3.25, 0.6};
                    armPosition = r.mapFunction(xVal,coord3,coord4);
                }
            }

            /*
            if (xVal > 2.693){
                double[] coord3 = {2.693,0.6616};
                double[] coord4 = {3.25, 0.6};
                armPosition = r.mapFunction(xVal,coord3,coord4);
            }

             */
            r.servo3.setPosition(armPosition);

            if (gamepad2.left_bumper) {
                r.servo4.setPosition(1.0);
            } else if (gamepad2.right_bumper) {
                r.servo4.setPosition(0.7);
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
                r.foundationMovePower(0);
            } else if (gamepad1.left_bumper) {
                r.foundationMovePower(1);
            }

            if (gamepad2.dpad_left) {
                //Remember to make it specfically set servo to a position
                //or have it run dissengage lift which is done by setting the boolean to true
               // r.ArmPosition(0.82);
                r.servo4.setPosition(0.6);
               // Thread.sleep(400);
                r.servo7.setPosition(0.85);
            } else if (gamepad2.dpad_right){
                r.servo7.setPosition(0.45);
            }

            if (gamepad2.y){
                //r.servo4.setPosition(1.0);
                //r.servo4.setPosition(0.6);
                r.ArmPosition(0.73);
                r.servo4.setPosition(1.0);
            }

            if (gamepad2.x){
                r.ArmPosition(0.9);
                Thread.sleep(200);
                r.servo4.setPosition(0.7);
            }

            if (gamepad2.b){
                r.ArmPosition(0.25);
            }

            ledControl();

           // liftDisengage();

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
        if (r.lift1.getPower() != 0 && !gamepad2.a){
            lockLift = false;
        }

        if (gamepad2.a){
            lockLift = true;
        }

        if (r.lift1.getPower() == 0 && lockLift){
            r.lift1.setPower(-0.3);
        }
    }

    public void ledControl(){
        if (LEDtime.seconds() > 60 && LEDtime.seconds() < 90){
            r.setLEDs(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
        } else if (LEDtime.seconds() > 90 && LEDtime.seconds() < 105){
            r.setLEDs(RevBlinkinLedDriver.BlinkinPattern.RED_ORANGE);
        } else if (LEDtime.seconds() > 105){
            r.setLEDs(RevBlinkinLedDriver.BlinkinPattern.STROBE_RED);
        } else {
            r.setLEDs(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE);
        }
    }

    public void liftDisengage() throws InterruptedException{

        if (gamepad1.dpad_down){
            disengadeLift = true;
        }

        if (disengadeLift) {
            r.servo4.setPosition(0.6);
            Thread.sleep(300);
            r.ArmPosition(0.75);
        }

        if (disengadeLift && r.ArmAngle.getVoltage() > 1.2 && !liftTimerLockOut){
            liftTime.reset();
            liftTimerLockOut = true;
        }

        if (disengadeLift && liftTime.milliseconds() > 2000){
            r.lift1.setPower(0);
            disengadeLift = false;
            liftTimerLockOut = false;
        }
    }
}

// Kazen's a nerd and he should feel bad