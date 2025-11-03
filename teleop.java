package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp(name = "DecodeMain")
public class DecodeMain extends LinearOpMode {
    private Blinker control_Hub;
  
    private DcMotor bottomL = null;
    private DcMotor bottomR = null;
    private DcMotor topL = null;
    private DcMotor topR = null;
    
    private DcMotor rightIntake = null;
    private DcMotor leftIntake = null;    
    private DcMotor leftOutake = null;
    private DcMotor rightOutake = null; 
    
    //private CRServo wrist = null;
    
    @Override
    public void runOpMode() {
        // Initialize hardware
        topL = hardwareMap.get(DcMotor.class, "topL");
        topR = hardwareMap.get(DcMotor.class, "topR");
        bottomL = hardwareMap.get(DcMotor.class, "bottomL");
        bottomR = hardwareMap.get(DcMotor.class, "bottomR");
        
        rightIntake = hardwareMap.get(DcMotor.class, "rightIntake");
        leftIntake = hardwareMap.get(DcMotor.class, "leftIntake");
        leftOutake = hardwareMap.get(DcMotor.class, "leftOutake");
        rightOutake = hardwareMap.get(DcMotor.class, "rightOutake");
        
        //wrist = hardwareMap.get(CRServo.class, "wrist");

        // Reverse motor direction
        bottomL.setDirection(DcMotor.Direction.REVERSE);
        topL.setDirection(DcMotor.Direction.REVERSE);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Normal movement and other functions
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1; // Adjust for strafing
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
                rightIntake.setPower(0.75);
                leftIntake.setPower(-0.75);
            } else {
                rightIntake.setPower(0);
                leftIntake.setPower(0);
            } 
            
            // Outtake 
            if (gamepad2.dpad_up) {
                rightOutake.setPower(0.99);
                leftOutake.setPower(-0.99);
            } else {
                rightOutake.setPower(0);
                leftOutake.setPower(0);
            }
            
            // Wrist
            //if (gamepad2.x) {
              //  wrist.setPower(0.5);
            //} else {
              //  wrist.setPower(0);
            //}
            
            telemetry.update();
        }
    }
}
