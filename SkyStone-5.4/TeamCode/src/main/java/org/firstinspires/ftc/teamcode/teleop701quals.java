package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Quals", group="Hardware Test")
@Disabled
public class teleop701quals extends LinearOpMode {

    boolean setOnce;
    double stopPos;
    double target;

    int level = 0;
    float mod = 1;
    float mod2 = 1;
    int liftPos = 0;
    int changeInEncoder;

    boolean lockLift = false;

    boolean liftdown = false;
    boolean slowdown = false;

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime time = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        Hardware r = new Hardware(this);

        waitForStart();
        runtime.reset();
        time.reset();

        while (opModeIsActive()) {

            //Change 2000 to whatever max position is
            double[] liftMin = {0, 1.0};
            double[] liftMax = {3000, 0.2};
           // mod2 = (float) (r.mapFunction(r.lift1.getCurrentPosition(), liftMin, liftMax));

           // r.servo1.setPower(-gamepad2.left_stick_y);
           // r.servo2.setPower(-gamepad2.left_stick_y);
            //0.88
            //double[] coord1 = {2.15, 0.92};
            //double[] coord2 = {0.156, 0.238};

            double[] coord1 = {2.145, 0.78};
            double[] coord2 = {0.203, 0.06};
            //r.servo3.setPosition(r.mapFunction(r.ArmAngle.getVoltage(), coord1, coord2));
            r.servo3.setPosition(Math.sin(r.ArmAngle.getVoltage()/2.45) + 0.029);

            if (gamepad2.x && runtime.milliseconds() > 240) {
               r.servo3.setPosition(0.03 + r.servo3.getPosition());
                runtime.reset();
            } else if (gamepad2.y && runtime.milliseconds() > 240) {
                r.servo3.setPosition(-0.03 + r.servo3.getPosition());
                runtime.reset();
            }


            if (gamepad2.left_bumper) {
                r.servo4.setPosition(0.5);
            } else if (gamepad2.right_bumper) {
                r.servo4.setPosition(0.09);
            }


            if (gamepad1.x){
                mod = 0.4f;
            } else if (gamepad1.y){
                mod = 1.0f;
            }

             /*

            if (gamepad2.x && time.milliseconds() > 750){
                mod = 0.4f;
                slowdown = true;
                time.reset();
            } else if (gamepad2.x && slowdown && time.milliseconds() > 750){
                mod = 1.0f;
                slowdown = false;
                time.reset();
            }

            if (mod == 1.0f){
                mod2 = (float) (r.mapFunction(r.lift1.getCurrentPosition(), liftMin, liftMax));
            } else {
                mod2 = 1;
            }
*/

            float LTMotorDC = gamepad1.left_stick_y - gamepad1.left_stick_x +  (0.5f) * gamepad1.right_stick_x;
            float LBMotorDC = gamepad1.left_stick_y + gamepad1.left_stick_x + (0.5f) * gamepad1.right_stick_x;
            float RTMotorDC = gamepad1.left_stick_y + gamepad1.left_stick_x - (0.5f) * gamepad1.right_stick_x;
            float RBMotorDC = gamepad1.left_stick_y - gamepad1.left_stick_x - (0.5f) * gamepad1.right_stick_x;

            LTMotorDC = LTMotorDC * 1f * mod * mod2;
            LBMotorDC = LBMotorDC * 1f * mod * mod2;
            RTMotorDC = RTMotorDC * 1f * mod * mod2;
            RBMotorDC = RBMotorDC * 1f * mod * mod2;

            float largestVal = 1.0f;
            largestVal = Math.max(largestVal, Math.abs(LTMotorDC));
            largestVal = Math.max(largestVal, Math.abs(LBMotorDC));
            largestVal = Math.max(largestVal, Math.abs(RTMotorDC));
            largestVal = Math.max(largestVal, Math.abs(RBMotorDC));

            LTMotorDC = LTMotorDC/largestVal;
            LBMotorDC = LBMotorDC/largestVal;
            RTMotorDC = RTMotorDC/largestVal;
            RBMotorDC = RBMotorDC/largestVal;

            r.OpenPower(LTMotorDC, LBMotorDC, RTMotorDC, RBMotorDC);

            r.intake1.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
            r.intake2.setPower(gamepad1.right_trigger - gamepad1.left_trigger);

            /*
            //r.lift1.setPower(gamepad2.right_stick_y);
            if (r.lift1.getCurrentPosition() >= -2300) {
                int changeInEncoder = (int) (gamepad2.right_stick_y * 40);
                liftPos = liftPos + changeInEncoder;
                r.lift1.setTargetPosition(liftPos);
                r.lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                r.lift1.setPower(1);
            } else if (r.lift1.getCurrentPosition() < -2300){
                liftPos = -2200;
                r.lift1.setTargetPosition(liftPos);
                r.lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                r.lift1.setPower(0.5);
            }
             */


          //  changeInEncoder = (int) (gamepad2.right_stick_y * 40);
/*
            if (r.lift1.getCurrentPosition() <= -2300 && changeInEncoder < 0) {
                changeInEncoder = 0;
            } else if (r.lift1.getCurrentPosition() >= 5 && changeInEncoder > 0) {
                changeInEncoder = 0;
            }

 */

/*

            liftPos = liftPos + changeInEncoder;
            r.lift1.setTargetPosition(liftPos);
            r.lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            r.lift1.setPower(1);

*/

            r.lift1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            r.lift1.setPower((gamepad2.left_trigger - gamepad2.right_trigger));

            if (r.lift1.getPower() != 0 && gamepad2.a){
                lockLift = false;
            }

            if (gamepad2.a){
                lockLift = true;
            }

            if (r.lift1.getPower() == 0 && lockLift){
                r.lift1.setPower(-0.125);
            }


            if (gamepad1.right_bumper){
                r.foundationMovePower(-1);
            } else if (gamepad1.left_bumper) {
                r.foundationMovePower(1);
            }
/*
            if (r.touch.isPressed() && !r.isRobotReversed()){
                r.reverseRobot();
            } else if (r.touch.isPressed() && r.isRobotReversed()){
                r.regularRobot();
            }

            if (gamepad1.a && r.isRobotReversed){
                r.regularRobot();
            } else if (gamepad1.a && !r.isRobotReversed){
                r.reverseRobot();
            }

 */
/*
            if (gamepad2.a){
                r.servo4.setPosition(0.3);
                Thread.sleep(500);
                r.ArmPower(-1);
                liftdown = true;
                runtime.reset();
            }


            if (liftdown){
                r.lift1.setPower(0.6);
                if (runtime.milliseconds() > 2000){
                    liftdown = false;
                }

            }

 */

            //2350

            /*
            if (gamepad2.x && runtime.milliseconds() > 240) {
                if (level < 6) {
                    level++;
                }
                r.lift1.setTargetPosition(200 * level);
                r.lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                r.lift1.setPower(1);
                runtime.reset();
            } else if (gamepad2.y && runtime.milliseconds() > 240) {
                if (level > 0) {
                    level--;
                }
                r.lift1.setTargetPosition(200 * level);
                r.lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                r.lift1.setPower(1);
                runtime.reset();
            }
            */

            if (gamepad2.dpad_left) {
                r.servo7.setPosition(0.85);
            } else if (gamepad2.dpad_right){
                r.servo7.setPosition(0.45);
            }

            if (r.servo7.getPosition() == 0.85 && r.touch.isPressed()){
                r.servo7.setPosition(0.45);
            }



            

            //If button is pressed dont do anything
            //If button is not pressed but was pressed in the last cycle

/*
            telemetry.addData("Potem", r.ArmAngle.getVoltage());
            telemetry.addData("servo1: ", r.servo1.getPower());
            telemetry.addData("servo2: ", r.servo2.getPower());
            telemetry.addData("servo3: ", r.servo3.getPosition());
            telemetry.addData("servo4: ", r.servo4.getPosition());
            telemetry.addData("level: ", level);
            telemetry.addData("Lift Pos: ", r.lift1.getCurrentPosition());
            telemetry.addData("Encoders L ", r.LTMotor.getCurrentPosition());
            telemetry.addData("Encoders C ", r.LBMotor.getCurrentPosition());
            telemetry.addData("Encoders R ", r.RTMotor.getCurrentPosition());
            telemetry.addData("Lift Power: ", r.lift1.getPower());
            telemetry.addData("Lift Target: ", r.lift1.getTargetPosition());
            telemetry.addData("Runtime: ", runtime.milliseconds());
           // telemetry.addData("LT Encoder: ", r.LTMotor.get)
            telemetry.update();
 */
            r.showTelemtry(this);

        }
    }
}
