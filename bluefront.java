package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import java.util.List;
import java.util.concurrent.TimeUnit;
import com.qualcomm.robotcore.hardware.Servo;


@Autonomous(name = "BlueFrontAutonomoussss.java")
public class BlueFrontAutonomous extends LinearOpMode {

    private Blinker control_Hub;
    private DcMotor bottomL = null;
    private DcMotor bottomR = null;
    private DcMotor topL = null;
    private DcMotor topR = null;
    private DcMotor intake = null;
  
    private DcMotor rightOutake = null;
    private DcMotor leftOutake = null;
    private Servo wrist = null;
    private CRServo wheel = null;
  
    private static final boolean USE_WEBCAM = true; // true if using Webcam 1
    private static final int[] DESIRED_TAG_IDS = {21, 22, 23};
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private int seenTagId = -1;

    private final double DESIRED_DISTANCE_INCHES = 12.0;
    private final double SPEED_GAIN = 0.02;
    private final double TURN_GAIN = 0.01;
    private final double MAX_AUTO_SPEED = 0.5;
    private final double MAX_AUTO_TURN = 0.35;
    private final double RANGE_TOLERANCE = 1.5;   // inches
    private final double BEARING_TOLERANCE = 4.0; // degrees

    private final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        telemetry.addLine("RunOpMode: start init");
        telemetry.update();

        // Initialize the hardware variables
        try {
            topL = hardwareMap.get(DcMotor.class, "topL");
            topR = hardwareMap.get(DcMotor.class, "topR");
            bottomL = hardwareMap.get(DcMotor.class, "bottomL");
            bottomR = hardwareMap.get(DcMotor.class, "bottomR");
          
            intake = hardwareMap.get(DcMotor.class, "intake");
            leftOutake = hardwareMap.get(DcMotor.class, "leftOutake");
            rightOutake = hardwareMap.get(DcMotor.class, "rightOutake");
            wheel = hardwareMap.get(CRServo.class, "wheel");
            wrist = hardwareMap.get(Servo.class, "wrist");

        } catch (Exception e) {
            telemetry.addData("ERROR", "hardwareMap lookup failed: %s", e.toString());
            telemetry.update();
            // don't return — we still want to see vision telemetry if possible
        }

        // Reverse motor direction where appropriate
        try {
            topL.setDirection(DcMotor.Direction.FORWARD);
            topR.setDirection(DcMotor.Direction.FORWARD);
            bottomL.setDirection(DcMotor.Direction.REVERSE);
        } catch (Exception e) {
            telemetry.addData("WARN", "couldn't set motor direction: %s", e.toString());
            telemetry.update();
        }

        // Reset encoders (safe guarded)
        try {
            if (topL != null) topL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            if (topR != null) topR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            if (bottomL != null) bottomL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            if (bottomR != null) bottomR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            if (topL != null) topL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            if (topR != null) topR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            if (bottomL != null) bottomL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            if (bottomR != null) bottomR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        } catch (Exception e) {
            telemetry.addData("WARN", "encoder/reset failed: %s", e.toString());
            telemetry.update();
        }
       
        topL.setDirection(DcMotor.Direction.FORWARD);
        topR.setDirection(DcMotor.Direction.FORWARD);
        bottomL.setDirection(DcMotor.Direction.REVERSE);

        // Init vision
        telemetry.addLine("initAprilTag()");
        telemetry.update();
        try {
            initAprilTag();
        } catch (Exception e) {
            telemetry.addData("ERROR", "initAprilTag failed: %s", e.toString());
            telemetry.update();
        }

        if (USE_WEBCAM) {
            telemetry.addLine("Setting manual exposure...");
            telemetry.update();
            // This could block if camera never streams; so we log progress inside
            setManualExposure(6, 250);
        }

        telemetry.addLine("Initialized. Scanning for desired tags (21/22/23) pre-start...");
        telemetry.update();

        // Pre-start sample
        sampleTagBeforeStart(1.5);

        telemetry.addData("Prestart seenTagId", seenTagId >= 0 ? seenTagId : "none");
        telemetry.addLine("Press PLAY to start the recenter test.");
        telemetry.update();

        waitForStart();
        runtime.reset();

        telemetry.addData("Status", "After start - Initialized");
        telemetry.addData("seenTagId (post-start)", seenTagId >= 0 ? seenTagId : "none");
        telemetry.update();

        if (!opModeIsActive()) {
            telemetry.addLine("opMode not active after start -> exit");
            telemetry.update();
            return;
        }
       
        //sleep(10000);
       
       // topL.setPower(-0.5);
        //topR.setPower(-0.5);
        //bottomL.setPower(-0.5);
        //bottomR.setPower(-0.5);
        //sleep(1000);
        //topL.setPower(0);
        //topR.setPower(0);
        //bottomL.setPower(0);
        //bottomR.setPower(0);
        driveForward(65,0.7);

        // Shoot - need to refine the shooting sequence
        //first ball
        rightOutake.setPower(0.60);
        leftOutake.setPower(-0.60);
        wrist.setPosition(0); // SHOOT
        sleep(3000);

        //second ball
        wrist.setPosition(0.8);
        sleep(1000);
        wheel.setPower(-0.60);
        sleep(1100);
        wheel.setPower(0);
        wrist.setPosition(0); // SHOOT
        sleep(3000);

        //third  ball
        wrist.setPosition(0.8);
        sleep(1000);
        wheel.setPower(-0.60);
        sleep(1100);
        wheel.setPower(0);
        wrist.setPosition(0); // shoot position
        sleep(3000);
       
        rightOutake.setPower(0);
        leftOutake.setPower(0);
        //wrist.setPosition(0.8);

        // Intake first row
   //     turnRight(100,0.5);
   //     driveForward(35,0.65);
      //  turnRight(90,0.5);
       
        // Shoot first row

        // If we didn't see a tag prestart, try scanning during run
        if (seenTagId < 0 || !isDesiredTag(seenTagId)) {
            telemetry.addLine("No desired prestart tag — scanning during run...");
            telemetry.update();

            long searchStart = System.currentTimeMillis();
            long searchTimeoutMs = 15000; // search up to 15s
            boolean foundDesired = false;
            int loopCount = 0;

            while (opModeIsActive()
                    && (System.currentTimeMillis() - searchStart) < searchTimeoutMs
                    && !foundDesired) {

                loopCount++;
                List<AprilTagDetection> detections = null;
                try {
                    detections = aprilTag == null ? null : aprilTag.getDetections();
                } catch (Exception e) {
                    telemetry.addData("vision-ex", "getDetections threw: %s", e.toString());
                    telemetry.update();
                    // short sleep and continue so we can see logs
                    sleep(50);
                    continue;
                }

                telemetry.addData("scan-loop", "%d, detections=%s", loopCount, detections == null ? "null" : detections.size());
                telemetry.update();

                if (detections != null && !detections.isEmpty()) {
                    stopAllDrivePower();
                    telemetry.addLine("Detections seen; processing...");
                    telemetry.update();

                    for (AprilTagDetection d : detections) {
                        if (d == null) continue;
                        telemetry.addData("seen", "%d (%s)", d.id, d.metadata != null ? d.metadata.name : "no-meta");
                        telemetry.update();

                        if (isDesiredTag(d.id)) {
                            seenTagId = d.id;
                            telemetry.addData("chosen", seenTagId);
                           
                            telemetry.update();
                            foundDesired = true;
                            break;
                        }
                    }

                    // Give the pipeline a moment
                    sleep(60);

                    if (foundDesired) break;
                } else {
                    // rotate slowly in-place — for safety we do not actually drive here unless
                    // you uncomment the setMecanumPower line.
                    telemetry.addLine("No tags seen — (virtual rotate)");
                    telemetry.update();
                    sleep(150);
                }
            } // scan while

            stopAllDrivePower();
            telemetry.addData("POST-SCAN seenTagId", seenTagId >= 0 ? seenTagId : "none");
            telemetry.update();
        } else {
            telemetry.addData("Info", "We saw a desired tag prestart: %d", seenTagId);
            telemetry.update();
        }

        // Now process whichever tag we have (prestart or scan)
        telemetry.addLine("Processing tag decision now...");
        telemetry.update();

        if (seenTagId >= 0 && isDesiredTag(seenTagId)) {
            telemetry.addData("Debug: Tag found, proceeding with ID", seenTagId);
            telemetry.update();

            if (seenTagId == 21) {
                telemetry.addLine("Debug: ID 21 IS SEEN! Calling moveGpp()");
                telemetry.update();
                moveGpp();
            } else if (seenTagId == 23) {
                telemetry.addLine("Debug: ID 23 IS SEEN! Calling movePpg()");
                telemetry.update();
                movePpg();
            } else if (seenTagId == 22) {
                telemetry.addLine("Debug: ID 22 IS SEEN! Calling movePgp()");
                telemetry.update();
                movePgp();
            } else {
                telemetry.addData("Debug: Unknown tag id after scan", seenTagId);
                telemetry.update();
            }
        } else {
            telemetry.addLine("Debug: Tag not found or not desired after scan.");
            telemetry.update();
        }
       
        // Final cleanup (printed after the check)
        telemetry.addData("END: Tag is", seenTagId);
        telemetry.addData("DBG - final runtime(s)", "%.3f", runtime.seconds());
        telemetry.update();
    }

    // ---- small helper movement functions ----
    private void moveGpp() {
        telemetry.addData("moveGpppppppppppppp", "called");
        telemetry.update();
        // Insert robot motion you want here. For debugging, we keep it simple.
        sleep(200);
        telemetry.addLine("moveGpp: done");
        telemetry.update();
    }

    private void movePpg() {
        telemetry.addData("movePpggggggggggggggg", "called");
        telemetry.update();
        sleep(200);
        telemetry.addLine("movePpg: done");
        telemetry.update();
    }

    private void movePgp() {
        telemetry.addData("movePgppppppppppppppppppp", "called");
        driveForward(5,0.5);
        telemetry.update();
        sleep(200);
        telemetry.addLine("movePgp: done");
        telemetry.update();
    }

    private boolean alignToTagAndCenter(int targetId, long timeoutMs) {
        long start = System.currentTimeMillis();
        while (opModeIsActive() && (System.currentTimeMillis() - start) < timeoutMs) {
            List<AprilTagDetection> detections = null;
            try {
                detections = aprilTag == null ? null : aprilTag.getDetections();
            } catch (Exception e) {
                telemetry.addData("align-getDet", "ex: %s", e.toString());
                telemetry.update();
            }

            AprilTagDetection found = null;
            if (detections != null) {
                for (AprilTagDetection d : detections) {
                    if (d == null) continue;
                    if (d.id == targetId && isDesiredTag(d.id)) {
                        found = d;
                        break;
                    }
                }
            }

            if (found == null) {
                setMecanumPower(0.0, 0.0, 0.005);
                telemetry.addData("Searching", "for tag %d (detections=%s)", targetId, detections == null ? "null" : detections.size());
                telemetry.update();
                sleep(40);
                continue;
            }

            double range = found.ftcPose.range;
            double bearing = found.ftcPose.bearing;

            double rangeError = range - DESIRED_DISTANCE_INCHES;
            double headingError = bearing;

            double drive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
            double turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);

            double strafe = 0.0;

            setMecanumPower(drive, strafe, turn);

            telemetry.addData("Tag", targetId);
            telemetry.addData("Range", "%.2f (err %.2f)", range, rangeError);
            telemetry.addData("Bearing", "%.2f (err %.2f)", bearing, headingError);
            telemetry.update();

            if (Math.abs(rangeError) <= RANGE_TOLERANCE && Math.abs(headingError) <= BEARING_TOLERANCE) {
                stopAllDrivePower();
                return true;
            }

            sleep(40);
        }

        stopAllDrivePower();
        return false;
    }

    private void setMecanumPower(double forward, double strafe, double rotate) {
        if (topL == null || topR == null || bottomL == null || bottomR == null) {
            // hardware missing — just log
            telemetry.addData("setMecanumPower", "missing motor(s)");
            telemetry.update();
            return;
        }

        double fl = forward + strafe + rotate;
        double fr = forward - strafe - rotate;
        double bl = forward - strafe + rotate;
        double br = forward + strafe - rotate;
        double max = Math.max(Math.max(Math.abs(fl), Math.abs(fr)), Math.max(Math.abs(bl), Math.abs(br)));
        if (max > 1.0) {
            fl /= max;
            fr /= max;
            bl /= max;
            br /= max;
        }

        topL.setPower(fl);
        topR.setPower(fr);
        bottomL.setPower(bl);
        bottomR.setPower(br);
    }

    public void driveForward(double inches, double power) {
        // unchanged but safe-guard motors
        if (topL == null || topR == null || bottomL == null || bottomR == null) {
            telemetry.addData("driveForward", "missing motor(s)");
            telemetry.update();
            return;
        }

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

    public void turnLeft(double degrees, double power) {
        if (topL == null || topR == null || bottomL == null || bottomR == null) {
            telemetry.addData("turnLeft", "missing motor(s)");
            telemetry.update();
            return;
        }

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

    public void turnRight(double degrees, double power) {
        if (topL == null || topR == null || bottomL == null || bottomR == null) {
            telemetry.addData("turnRight", "missing motor(s)");
            telemetry.update();
            return;
        }

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

    private void stopMotors() {
        if (topL != null) topL.setPower(0);
        if (topR != null) topR.setPower(0);
        if (bottomL != null) bottomL.setPower(0);
        if (bottomR != null) bottomR.setPower(0);

        if (topL != null) topL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (topR != null) topR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (bottomL != null) bottomL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (bottomR != null) bottomR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private int inchesToTicks(double inches) {
        final double TICKS_PER_REV = 537.6;
        final double WHEEL_DIAMETER = 4.0;
        final double CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;
        return (int) ((inches / CIRCUMFERENCE) * TICKS_PER_REV);
    }

    private int degreesToTicks(double degrees) {
        final double ROBOT_DIAMETER = 18.0;
        final double ROBOT_CIRCUMFERENCE = Math.PI * ROBOT_DIAMETER;
        double distance = (degrees / 360.0) * ROBOT_CIRCUMFERENCE;
        return inchesToTicks(distance);
    }

    // ------------
    // Vision helpers
    // ------------
    private void initAprilTag() {
        aprilTag = new AprilTagProcessor.Builder().build();
        aprilTag.setDecimation(2);

        if (USE_WEBCAM) {
            try {
                visionPortal = new VisionPortal.Builder()
                        .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                        .addProcessor(aprilTag)
                        .build();
            } catch (Exception e) {
                telemetry.addData("vision-init", "webcam init failed: %s", e.toString());
                telemetry.update();
                visionPortal = null;
            }
        } else {
            try {
                visionPortal = new VisionPortal.Builder()
                        .setCamera(BuiltinCameraDirection.BACK)
                        .addProcessor(aprilTag)
                        .build();
            } catch (Exception e) {
                telemetry.addData("vision-init", "phone cam init failed: %s", e.toString());
                telemetry.update();
                visionPortal = null;
            }
        }

        telemetry.addData("visionPortal", visionPortal == null ? "null" : "created");
        telemetry.update();
    }

    private void sampleTagBeforeStart(double timeoutSeconds) {
        double start = runtime.seconds();
        telemetry.addData("sampleTagBeforeStart", "timeout=%.2fs", timeoutSeconds);
        telemetry.update();

        while (!isStarted() && !isStopRequested() && (runtime.seconds() - start) < timeoutSeconds) {
            List<AprilTagDetection> detections = null;
            try {
                detections = aprilTag == null ? null : aprilTag.getDetections();
            } catch (Exception e) {
                telemetry.addData("pre-getDet", "ex: %s", e.toString());
                telemetry.update();
                sleep(40);
                continue;
            }

            telemetry.addData("pre-detections", detections == null ? "null" : detections.size());
            telemetry.update();

            if (detections != null && !detections.isEmpty()) {
                for (AprilTagDetection det : detections) {
                    if (det == null) continue;
                    telemetry.addData("preseen", "%d (%s)", det.id, det.metadata != null ? det.metadata.name : "no-meta");
                    telemetry.update();

                    if (det.metadata != null && isDesiredTag(det.id)) {
                        seenTagId = det.id;
                        telemetry.addData("prechosen", seenTagId);
                        telemetry.update();
                        return;
                    }
                }
            }

            sleep(50);
        }
        telemetry.addData("prestart-end", "done scanning prestart");
        telemetry.update();
    }

    private boolean isDesiredTag(int id) {
        for (int x : DESIRED_TAG_IDS) if (x == id) return true;
        return false;
    }

    private void stopAllDrivePower() {
        setMecanumPower(0.0, 0.0, 0.0);
    }

    private void setManualExposure(int exposureMS, int gain) {
        if (visionPortal == null) {
            telemetry.addData("setManualExposure", "visionPortal == null");
            telemetry.update();
            return;
        }

        telemetry.addData("CameraState-at-call", visionPortal.getCameraState().toString());
        telemetry.update();

        long start = System.currentTimeMillis();
        while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) && (System.currentTimeMillis()-start) < 5000) {
            telemetry.addData("Camera", "Waiting for stream (timeout 5s) state=%s", visionPortal.getCameraState());
            telemetry.update();
            sleep(50);
        }

        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "NOT STREAMING after wait, continuing without exposure set");
            telemetry.update();
            return;
        }

        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
        if (exposureControl != null) {
            try {
                if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                    exposureControl.setMode(ExposureControl.Mode.Manual);
                    sleep(50);
                }
                exposureControl.setExposure((long) exposureMS, TimeUnit.MILLISECONDS);
                sleep(20);
            } catch (Exception e) {
                telemetry.addData("exposure", "failed: %s", e.toString());
                telemetry.update();
            }
        } else {
            telemetry.addData("exposure", "control null");
            telemetry.update();
        }

        GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
        if (gainControl != null) {
            try {
                gainControl.setGain(gain);
                sleep(20);
            } catch (Exception e) {
                telemetry.addData("gain", "failed: %s", e.toString());
                telemetry.update();
            }
        } else {
            telemetry.addData("gain", "control null");
            telemetry.update();
        }

        telemetry.addData("Camera", "Ready");
        telemetry.update();
    }
}
