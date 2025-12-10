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
    
    private CRServo wheel = null;
    
    public ElapsedTime runtime = new ElapsedTime();
    public boolean buttonWasPressed = false;
    public double reverseStartTime = 0.0;
    public final double REVERSE_DURATION = 0.5;
    
    private Servo wrist = null;
    private Servo wheel = null;
    
    private str blinker = "orange"
    @Override
    public void runOpMode() {
        // Initialize hardware
        control_hub = hardwareMap.get(Blinker.class, "Control Hub");
        
        topL = hardwareMap.get(DcMotor.class, "topL");
        topR = hardwareMap.get(DcMotor.class, "topR");
        bottomL = hardwareMap.get(DcMotor.class, "bottomL");
        bottomR = hardwareMap.get(DcMotor.class, "bottomR");
        
        intake = hardwareMap.get(DcMotor.class, "intake");
        wrist = hardwareMap.get(Servo.class, "wrist");
        wheel = hardwareMap.get(Servo.class, "wheel");

        // Reverse motor direction
        bottomL.setDirection(DcMotor.Direction.REVERSE);
        topL.setDirection(DcMotor.Direction.REVERSE);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Normal movement and other functions
            double y = gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * -1.1; // Adjust for strafing
            double rx = gamepad1.right_stick_x;
            double maxPower = 0.5;
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
            if (gamepad2.y) {
                intake.setPower(-1);
            } else {
                intake.setPower(0);
            }
            
            //Wheel
            // manual changing
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
                sleep(550); // CHANGE THIS FOR 60 DEGREES
                wheel.setPower(0);
            } else if (gamepad2.b) {
                wheel.setPower(-0.2);
                sleep(550);
                wheel.setPower(0);
            }

            boolean status = false;
            // Outtake 
            if (gamepad2.dpad_up) {
                rightOutake.setPower(0.6); //change power
                leftOutake.setPower(-0.6);
                buttonWasPressed = true;
            } else if (buttonWasPressed) {
                if (runtime.seconds() < reverseStartTime + REVERSE_DURATION) {
                    rightOutake.setPower(-0.1);
                    leftOutake.setPower(0.1);
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

            // Wrist
            if (gamepad2.x) {
                telemetry.addData("Position", wrist.getPosition());
                wrist.setPosition(0);
            } else {
                telemetry.addData("Position", wrist.getPosition());
                wrist.setPosition(0.8);
            }
            
            // Wheel
            if (gamepad2.dpad_left) {
                wheel.setPosition(0);
            } else if (gamepad2.dpad_right) {
                wheel.setPosition(1);
            }

            // wyatt modified Outtake 
            if (gamepad2.dpad_up) {
                if (blinker == "orange") {
                    rightOutake.setPower(0.99);
                    leftOutake.setPower(-0.99);
                } else if (blinker == "red") {
                    rightOutake.setPower(0.99);
                    leftOutake.setPower(-0.99);
                } else {
                    rightOutake.setPower(0.99);
                    leftOutake.setPower(-0.99);
            } else {
                rightOutake.setPower(0);
                leftOutake.setPower(0);
            }

            if (blinker == "orange") {
                control_hub.setConstant(Color.ORANGE);
            } else if (blinker == "red") {
                control_hub.setConstant(Color.RED);
            } else {
                control_hub.setConstant(Color.GREEN);
            }
                
            //crazy button
            if (gamepad1.a) {
                if (blinker == "orange") {
                    str blinker = "red";
                } else if (blinker == "red) {
                    str blinker = "green";
                } else {
                    str blinker = "orange";
                }
            }
            
            telemetry.update();
        }
    }
}
