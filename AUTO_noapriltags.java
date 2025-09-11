// This code will be made for the BLUE (left) team, starting position to the left of the white tape

//WORKING! USE ON MATCH DAY!
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "BlueBack_Autonomous.java")
public class BlueBack_Autonomous extends LinearOpMode {

    private Blinker control_Hub;
    private DcMotor bottomL = null;
    private DcMotor bottomR = null;
    private DcMotor topL = null;
    private DcMotor topR = null;

    @Override
    public void runOpMode() {
        // Initialize the hardware variables
        topL = hardwareMap.get(DcMotor.class, "topL");
        topR = hardwareMap.get(DcMotor.class, "topR");
        bottomL = hardwareMap.get(DcMotor.class, "bottomL");
        bottomR = hardwareMap.get(DcMotor.class, "bottomR");

        // Reverse motor direction
        bottomL.setDirection(DcMotor.Direction.REVERSE);
        topL.setDirection(DcMotor.Direction.REVERSE);
        topR.setDirection(DcMotor.Direction.REVERSE); 

        // Set all motors to use encoders
        topL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        topR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottomL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottomR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        topL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        topR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bottomL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bottomR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            sleep(10); // waits for 10 milliseconds 
          
            // Drive to high basket 
            driveForward(3,0.5);
            turnLeft(160,0.5);
            driveForward(70,0.5);
            turnLeft(75,0.5);
            driveForward(10,0.5);

        }
    }

    // Method to drive forward or backward a given distance (in inches)
    public void driveForward(double inches, double power) {
        int ticks = inchesToTicks(inches);

        topL.setTargetPosition(topL.getCurrentPosition() + ticks);
        topR.setTargetPosition(topR.getCurrentPosition() + ticks);
        bottomL.setTargetPosition(bottomL.getCurrentPosition() + ticks);
        bottomR.setTargetPosition(bottomR.getCurrentPosition() + ticks);

        topL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        topR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bottomL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bottomR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        topL.setPower(power);
        topR.setPower(power);
        bottomL.setPower(power);
        bottomR.setPower(power);

        while (opModeIsActive() && topL.isBusy() && topR.isBusy() && bottomL.isBusy() && bottomR.isBusy()) {
            telemetry.addData("Path", "Driving Forward");
            telemetry.update();
        }

        stopMotors();
    }

    // Method to turn left a given angle (in degrees)
    public void turnLeft(double degrees, double power) {
        int ticks = degreesToTicks(degrees);

        topL.setTargetPosition(topL.getCurrentPosition() - ticks);
        topR.setTargetPosition(topR.getCurrentPosition() + ticks);
        bottomL.setTargetPosition(bottomL.getCurrentPosition() - ticks);
        bottomR.setTargetPosition(bottomR.getCurrentPosition() + ticks);

        topL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        topR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bottomL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bottomR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        topL.setPower(power);
        topR.setPower(power);
        bottomL.setPower(power);
        bottomR.setPower(power);

        while (opModeIsActive() && topL.isBusy() && topR.isBusy() && bottomL.isBusy() && bottomR.isBusy()) {
            telemetry.addData("Path", "Turning Left");
            telemetry.update();
        }

        stopMotors();
    }

    // Method to turn right a given angle (in degrees)
    public void turnRight(double degrees, double power) {
        int ticks = degreesToTicks(degrees);

        topL.setTargetPosition(topL.getCurrentPosition() + ticks);
        topR.setTargetPosition(topR.getCurrentPosition() - ticks);
        bottomL.setTargetPosition(bottomL.getCurrentPosition() + ticks);
        bottomR.setTargetPosition(bottomR.getCurrentPosition() - ticks);

        topL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        topR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bottomL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bottomR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        topL.setPower(power);
        topR.setPower(power);
        bottomL.setPower(power);
        bottomR.setPower(power);

        while (opModeIsActive() && topL.isBusy() && topR.isBusy() && bottomL.isBusy() && bottomR.isBusy()) {
            telemetry.addData("Path", "Turning Right");
            telemetry.update();
        }

        stopMotors();
    }

    // Method to stop all motors
    private void stopMotors() {
        topL.setPower(0);
        topR.setPower(0);
        bottomL.setPower(0);
        bottomR.setPower(0);

        topL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        topR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bottomL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bottomR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // Helper method to convert inches to encoder ticks
    private int inchesToTicks(double inches) {
        final double TICKS_PER_REV = 537.6;  // Example: GoBILDA 5202 motor
        final double WHEEL_DIAMETER = 4.0;  // In inches
        final double CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;
        return (int) ((inches / CIRCUMFERENCE) * TICKS_PER_REV);
    }

    // Helper method to convert degrees to encoder ticks
    private int degreesToTicks(double degrees) {
        final double ROBOT_DIAMETER = 18.0;  // Example robot diameter in inches
        final double ROBOT_CIRCUMFERENCE = Math.PI * ROBOT_DIAMETER;
        double distance = (degrees / 360.0) * ROBOT_CIRCUMFERENCE;
        return inchesToTicks(distance);
    }
}
