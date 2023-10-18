package org.firstinspires.ftc.teamcode;

//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="PID Testing", group="Hardware Test")
//@Config
@Disabled
public class PID_Testing extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    public static int target = 0;
    boolean pressedDown;
    public static double P = 2.5;
    public static double I = 0.1;
    public static double D = 0.2;
    int liftPos = 0;
    int changeInEncoder;

   // FtcDashboard dashboard = FtcDashboard.getInstance();

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        Hardware r = new Hardware(this);

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {


            /*
            changeInEncoder = (int) (gamepad2.right_stick_y * 40);
            if (r.lift1.getCurrentPosition() <= -2300 && changeInEncoder < 0) {
                changeInEncoder = 0;
            } else if (r.lift1.getCurrentPosition() >= 5 && changeInEncoder > 0) {
                changeInEncoder = 0;
            }

             */

            //liftPos = liftPos + changeInEncoder;
            r.lift1.setTargetPosition(target);
            r.lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);




            telemetry.addData("P,I,D (orig)", "%.04f, %.04f, %.04f",
                    P, I, D);
            telemetry.update();


          //  TelemetryPacket packet = new TelemetryPacket();
          //  packet.put("x", 3.7);

         //   dashboard.sendTelemetryPacket(packet);


            /*
            r.lift1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            changeInEncoder = (int) ((-gamepad2.left_trigger + gamepad2.right_trigger) * 10);
            if (r.lift1.getCurrentPosition() <= -2300 && changeInEncoder < 0) {
                changeInEncoder = 0;
            } else if (r.lift1.getCurrentPosition() >= 5 && changeInEncoder > 0) {
                changeInEncoder = 0;
            }

            target = target + changeInEncoder;

            float error = target - r.lift1.getCurrentPosition();
            float power = P * error;
            power = Range.clip(power, -1, 1);
            Log.e("HI", "power is: " + power);
            r.lift1.setPower(power);

            if (gamepad2.a) {
                pressedDown = true;
            } else if (pressedDown) {
                P += 0.001f;
                pressedDown = false;
            }

            if (gamepad2.b) {
                pressedDown = true;
            } else if (pressedDown) {
                P -= 0.001f;
                pressedDown = false;
            }

            telemetry.addData("P: ", P);
            telemetry.addData("power: ", power);
            telemetry.addData("target: ", target);
            telemetry.addData("error: ", error);
            telemetry.update();

            //r.showTelemtry(this);
            */
        }
    }
}
