package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name = "DecodeMain")
public class DecodeMain extends LinearOpMode {
    private Blinker control_Hub;
  
    
    private DcMotor bottomL = null;
    private DcMotor bottomR = null;
    private DcMotor topL = null;
    private DcMotor topR = null;
    
    
    private DcMotor intake = null;
    private DcMotor leftOutake = null; 
    private DcMotor rightOutake = null; 
    
    private CRServo wheel = null;
    private Servo wrist = null;
    
    public ElapsedTime runtime = new ElapsedTime();
    public boolean buttonWasPressed = false;
    public double reverseStartTime = 0.0;
    public final double REVERSE_DURATION = 0.5;
    
    @Override
    public void runOpMode() {
        // Initialize hardware
        topL = hardwareMap.get(DcMotor.class, "topL");
        topR = hardwareMap.get(DcMotor.class, "topR");
        bottomL = hardwareMap.get(DcMotor.class, "bottomL");
        bottomR = hardwareMap.get(DcMotor.class, "bottomR");
        
        intake = hardwareMap.get(DcMotor.class, "intake");
        leftOutake = hardwareMap.get(DcMotor.class, "leftOutake");
        rightOutake = hardwareMap.get(DcMotor.class, "rightOutake");
        
        wrist = hardwareMap.get(Servo.class, "wrist");
        wheel = hardwareMap.get(CRServo.class, "wheel");

        
        topL.setDirection(DcMotor.Direction.FORWARD);
        topR.setDirection(DcMotor.Direction.REVERSE);
        bottomL.setDirection(DcMotor.Direction.FORWARD);
        telemetry.addData("Status", "Initialized");
        
        
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            
            double y = gamepad1.left_stick_y;
            double x = -gamepad1.left_stick_x * 1.1; // Adjust for strafing
            double rx = -gamepad1.right_stick_x;
            
            double maxPower = 0.7;
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator * maxPower;
            double backLeftPower = (y - x + rx) / denominator * maxPower;
            double frontRightPower = (y - x - rx) / denominator * maxPower;
            double backRightPower = (y + x - rx) / denominator * maxPower;

            topL.setPower(frontLeftPower);
            bottomL.setPower(backLeftPower);
            topR.setPower(frontRightPower);
            bottomR.setPower(backRightPower); 
            
            
            // Intake 
            if (gamepad2.a) {
                intake.setPower(1);
            } else if (gamepad2.dpad_down) {
                intake.setPower(-1);
            } else {
                intake.setPower(0);
            }
            
            
            // Outtake 
            if (gamepad2.dpad_up) {
                rightOutake.setPower(-0.8); //change power
                leftOutake.setPower(0.8);
                buttonWasPressed = true;
            } else if (buttonWasPressed) {
               if (runtime.seconds() < reverseStartTime + REVERSE_DURATION) {
                    rightOutake.setPower(0.1);
                    leftOutake.setPower(-0.1);
                } else {
                    rightOutake.setPower(0);
                    leftOutake.setPower(0);
                    buttonWasPressed = false;
                }
            } else {
                rightOutake.setPower(0);
                leftOutake.setPower(0);
                reverseStartTime = runtime.seconds();
            }
            
            // wheel manual changing
            if (gamepad2.dpad_left) {
                wheel.setPower(-.2);
            } else if (gamepad2.dpad_right) {
                wheel.setPower(.2);
            } else {
                wheel.setPower(0);
            }
            
            //timed-based changing
            if (gamepad2.x) {
                wheel.setPower(0.2);
                sleep(1125); // CHANGE THIS FOR 60 DEGREES. works as of 1/16/26
                wheel.setPower(0);
            } else if (gamepad2.b) {
                wheel.setPower(-0.2);
                sleep(1125);
                wheel.setPower(0);
            } else {
                wheel.setPower(0);
            }
            
            // Wrist
            if (gamepad2.y) {
                wrist.setPosition(1); //need to ADJUST based on ideal wrist launch position
            } else {
                wrist.setPosition(0); //may need to ADJUST
            }
            
            telemetry.update();
        }
    }
}
