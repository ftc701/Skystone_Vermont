package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;


public class Hardware {

    public DcMotor LTMotor;
    public DcMotor LBMotor;
    public DcMotor RTMotor;
    public DcMotor RBMotor;

    public DcMotor intake1;
    public DcMotor intake2;

    public DcMotorEx lift1;

    public DcMotor odoL;
    public DcMotor odoC;
    public DcMotor odoR;

    //public CRServo servo1;
    //public CRServo servo2;

    public Servo servo1;
    public Servo servo2;

    public Servo servo3;
    public Servo servo4;

    public Servo servo5;
    public Servo servo6;

    public Servo servo7;

    public AnalogInput ArmAngle;

    public ColorSensor sensorColor;
    public DistanceSensor sensorDistance;

    public TouchSensor touch;

    RevBlinkinLedDriver blinkinLedDriver;
    RevBlinkinLedDriver.BlinkinPattern pattern;

    public boolean isRobotReversed = false;

    //public C

    VisionPipeline vision;

    HardwareMap hardwareMap;

    public BNO055IMU imu;

    Orientation angles;

    public Hardware(OpMode opMode){
        hardwareMap = opMode.hardwareMap;
        init();
    }

    public void init() {


        LTMotor = hardwareMap.get(DcMotor.class, "LTMotor");
        LBMotor = hardwareMap.get(DcMotor.class, "LBMotor");
        RTMotor = hardwareMap.get(DcMotor.class, "RTMotor");
        RBMotor = hardwareMap.get(DcMotor.class, "RBMotor");

        intake1 = hardwareMap.get(DcMotor.class, "intake1");
        intake2 = hardwareMap.get(DcMotor.class, "intake2");

        lift1 = hardwareMap.get(DcMotorEx.class, "lift1");

        //odoL = hardwareMap.get(DcMotorEx.class, "odoL");
        odoC = hardwareMap.get(DcMotorEx.class, "intake1");
        odoR = hardwareMap.get(DcMotorEx.class, "intake2");

        servo1 = hardwareMap.get(Servo.class, "servo1");
        servo2 = hardwareMap.get(Servo.class, "servo2");
        servo3 = hardwareMap.get(Servo.class, "servo3");
        servo4 = hardwareMap.get(Servo.class, "servo4");

        servo5 = hardwareMap.get(Servo.class, "servo5");
        servo6 = hardwareMap.get(Servo.class, "servo6");

        servo7 = hardwareMap.get(Servo.class, "servo7");

        ArmAngle = hardwareMap.get(AnalogInput.class, "poten");

        sensorColor = hardwareMap.get(ColorSensor.class, "sense");
        sensorDistance = hardwareMap.get(DistanceSensor.class, "sense");

        touch = hardwareMap.get(TouchSensor.class, "touch");

        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        pattern = RevBlinkinLedDriver.BlinkinPattern.BLACK;
        blinkinLedDriver.setPattern(pattern);

        LTMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RTMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LTMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LBMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RTMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RBMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        LTMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LBMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RTMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RBMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        /*
        odoL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        odoC.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        odoR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        odoL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        odoC.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        odoR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
*/

        RTMotor.setDirection(DcMotor.Direction.REVERSE);
        RBMotor.setDirection(DcMotor.Direction.REVERSE);


        lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }

    public void OpenPower(double p1, double p2, double p3, double p4) {
        LTMotor.setPower(p1);
        LBMotor.setPower(p2);
        RTMotor.setPower(p3);
        RBMotor.setPower(p4);
    }

    public double mapFunction(double xVal, double[] coord1, double[] coord2)  {

        double x1 = coord1[0];
        double x2 = coord2[0];

        double y1 = coord1[1];
        double y2 = coord2[1];

        //0.75, 2.112
        //0, 0.045
        //y  = ((0.75 - 0)/(2.112 - 0.045) * (x - 0.045)
        //y  = (y1 - y2)/(x1 - x2) * (xVal - x2) + y2
        //y = mx + b
        //b = (y1) - (y1 - y2)/(x1 - x2) * x1
        //y = ((y1 - y2)/(x1 - x2)) * r.ArmAngle.getVoltage + ((y1) - (y1 - y2)/(x1 - x2) * x1)

        return ((y1 - y2)/(x1 - x2)) * xVal + ((y1) - (y1 - y2)/(x1 - x2) * x1);
    }

    public double ArmQuadraticFunction(double xVal){
        return (-0.240171) * Math.pow(xVal,2) + 0.898547*xVal + 0.101543;
    }

    public int SkyStoneLocation(){
        int location = 3; //Default

        int L = vision.getMassL();
        int C = vision.getMassC();
        int R = vision.getMassR();

        if (L > C && L > R){
            location = 1;
        } else if (C > L && C > R){
            location = 2;
        } else if (R > C && R > L){
            location = 3;
        }

        return location;
    }

    public void startVision(String alliance){

        vision = new VisionPipeline(alliance);

        OpenCvCamera phoneCam;

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        phoneCam.openCameraDevice();

        phoneCam.setPipeline(vision);

        phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
    }

    public void showTelemtry(LinearOpMode l){
        l.telemetry.addData("DesiredArmAngle", (0.680135/(1+82.009 * Math.pow(Math.E,-3.47386 * ArmAngle.getVoltage())))+0.135);
        l.telemetry.addData("Button Pressed:", touch.isPressed());
        l.telemetry.addData("Potem", ArmAngle.getVoltage());
        l.telemetry.addData("servo1: ", servo1.getPosition());
        l.telemetry.addData("servo2: ", servo2.getPosition());
        l.telemetry.addData("servo3: ", servo3.getPosition());
        l.telemetry.addData("servo4: ", servo4.getPosition());
        l.telemetry.addData("Lift Pos: ", lift1.getCurrentPosition());
       // l.telemetry.addData("Encoders L ", odoL.getCurrentPosition());
       // l.telemetry.addData("Encoders C ", odoC.getCurrentPosition());
       // l.telemetry.addData("Encoders R ", odoR.getCurrentPosition());
        l.telemetry.addData("Lift Power: ", lift1.getPower());
        l.telemetry.addData("Lift Target: ", lift1.getTargetPosition());
        l.telemetry.addData("LT Encoder: ", LTMotor.getCurrentPosition());
        l.telemetry.addData("LB Encoder: ", LBMotor.getCurrentPosition());
        l.telemetry.addData("RT Encoder: ", RTMotor.getCurrentPosition());
        l.telemetry.addData("RB Encoder: ", RBMotor.getCurrentPosition());
        l.telemetry.addData("Motor Powers: ", "%.2f,%.2f,%.2f,%.2f", LTMotor.getPower(), LBMotor.getPower(), RTMotor.getPower(), RBMotor.getPower());
        l.telemetry.update();
    }

    public void liftProportional(int target){
        //errors from 0 to 2100
        float error = target - lift1.getCurrentPosition();
        float P = 0.001f;
        float power = P * error;
        power = Range.clip(power, -1, 1);
        Log.e("HI", "power is: " + power);
        lift1.setPower(power);
    }

    public double getPotenAngle(){

        //Need to change some values
        double[] coord1 = {2.145, 180};
        double[] coord2 = {0.203, -180};

        return mapFunction(ArmAngle.getVoltage(),coord1,coord2);

    }

    public void changePID(DcMotorEx motor, double P, double I, double D){
        PIDCoefficients pidNew = new PIDCoefficients(P, I, D);
        motor.setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidNew);
        PIDCoefficients pidModified = motor.getPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void GyroTurnSimple(int TargetAngle){

        float testerror = (getHeading() - TargetAngle);

        // -1 < testerror < 1

        while (!((-1.5 < testerror) && (testerror < 1.5))){

            float propControl = 0.001f;

            float powerVal = (propControl * testerror);
            powerVal = Range.clip(powerVal,-1 ,1);
            OpenPower(-powerVal, -powerVal, powerVal, powerVal);

            testerror = (getHeading() - TargetAngle);

            //servo 4 is the servo that rotates the grabbing mechanism
            servo4.setPosition((-0.798/(0.566 - 3.43)) * ArmAngle.getVoltage() - 0.158);





        }
        OpenPower(0,0,0,0);
    }

    public float getHeading() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }

    public void TankForward(double power, int target){
        LTMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RTMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        if (target != 0) {

            RBMotor.setTargetPosition(target);
            RTMotor.setTargetPosition(target);
            LBMotor.setTargetPosition(target);
            LTMotor.setTargetPosition(target);

            LTMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            LBMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RTMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RBMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while (LTMotor.isBusy() && RTMotor.isBusy()) {
                OpenPower(power, power, power, power);
            }
            RobotStop();
        } else {

            LTMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            LBMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            RTMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            RBMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            RTMotor.setPower(power);
            RBMotor.setPower(power);
            LTMotor.setPower(power);
            LBMotor.setPower(power);
        }
    }

    public void TankTurn(double power, int target){
        LTMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RTMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        if (target != 0) {

            RBMotor.setTargetPosition(-target);
            RTMotor.setTargetPosition(-target);
            LBMotor.setTargetPosition(target);
            LTMotor.setTargetPosition(target);

            LTMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            LBMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RTMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RBMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while (LTMotor.isBusy() && RTMotor.isBusy()) {
                OpenPower(power, power, power, power);
            }
            RobotStop();
        } else {

            LTMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            LBMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            RTMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            RBMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            RTMotor.setPower(-power);
            RBMotor.setPower(-power);
            LTMotor.setPower(power);
            LBMotor.setPower(power);
        }
    }

    public void MecanaumStrafe(double power, int target){
        LTMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RTMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        if (target != 0) {

            RBMotor.setTargetPosition(target);
            RTMotor.setTargetPosition(-target);
            LBMotor.setTargetPosition(-target);
            LTMotor.setTargetPosition(target);

            LTMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            LBMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RTMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RBMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while (LTMotor.isBusy() && RTMotor.isBusy()) {
                OpenPower(power, power, power, power);
            }
            RobotStop();
        } else {

            LTMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            LBMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            RTMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            RBMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            RTMotor.setPower(power);
            RBMotor.setPower(-power);
            LTMotor.setPower(-power);
            LBMotor.setPower(power);
        }
    }

    public void RobotStop(){
        LTMotor.setPower(0);
        LBMotor.setPower(0);
        RTMotor.setPower(0);
        RBMotor.setPower(0);
    }

    public void IntakePower(double power){
        intake1.setPower(power);
        intake2.setPower(power);
    }

    public void ArmPower(double power){
       // servo1.setPower(power);
       // servo2.setPower(power);
    }

    public void ArmPosition(double position){
        servo1.setPosition(position);
        servo2.setPosition(position);
    }

    public void foundationMovePower(double position){
        servo5.setPosition(1 - position);
        servo6.setPosition(position);
    }

    public void reverseRobot(){

        LTMotor.setDirection(DcMotor.Direction.REVERSE);
        LBMotor.setDirection(DcMotor.Direction.REVERSE);
        RTMotor.setDirection(DcMotor.Direction.FORWARD);
        RBMotor.setDirection(DcMotor.Direction.FORWARD);

        isRobotReversed = true;
    }

    public void regularRobot(){

        LTMotor.setDirection(DcMotor.Direction.FORWARD);
        LBMotor.setDirection(DcMotor.Direction.FORWARD);
        RTMotor.setDirection(DcMotor.Direction.REVERSE);
        RBMotor.setDirection(DcMotor.Direction.REVERSE);

        isRobotReversed = false;
    }

    public boolean isRobotReversed() {

        return isRobotReversed;

    }

    public void setLEDs(RevBlinkinLedDriver.BlinkinPattern pattern){
        blinkinLedDriver.setPattern(pattern);
    }

}


