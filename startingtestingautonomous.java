// simple testing code that just scans for the ID, the nrecenters to be in teh middle of the ID

package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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

@Autonomous(name = "Test_AprilTag_Recenter")
public class Test_AprilTag_Recenter extends LinearOpMode {
    private DcMotor topL = null;
    private DcMotor topR = null;
    private DcMotor bottomL = null;
    private DcMotor bottomR = null;

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
        topL = hardwareMap.get(DcMotor.class, "topL");
        topR = hardwareMap.get(DcMotor.class, "topR");
        bottomL = hardwareMap.get(DcMotor.class, "bottomL");
        bottomR = hardwareMap.get(DcMotor.class, "bottomR");

        bottomL.setDirection(DcMotor.Direction.REVERSE);
        topL.setDirection(DcMotor.Direction.REVERSE);
        topR.setDirection(DcMotor.Direction.FORWARD);

        topL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        topR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottomL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottomR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        topL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        topR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bottomL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bottomR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        topL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        topR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bottomL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bottomR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        initAprilTag();
        if (USE_WEBCAM) setManualExposure(6, 250);

        telemetry.addLine("Initialized. Scanning for desired tags (21/22/23) pre-start...");
        telemetry.update();
        sampleTagBeforeStart(1.5);

        telemetry.addData("Prestart seenTagId", seenTagId >= 0 ? seenTagId : "none");
        telemetry.addLine("Press PLAY to start the recenter test.");
        telemetry.update();

        waitForStart();
        runtime.reset();

        if (opModeIsActive()) {
            if (seenTagId < 0 || !isDesiredTag(seenTagId)) {
                telemetry.addLine("No desired prestart tag — searching during run...");
                telemetry.update();

                long searchStart = System.currentTimeMillis();
                long searchTimeoutMs = 8000; // search up to 8s
                while (opModeIsActive() && (System.currentTimeMillis() - searchStart) < searchTimeoutMs && !isDesiredTag(seenTagId)) {
                    List<AprilTagDetection> detections = aprilTag.getDetections();
                    for (AprilTagDetection d : detections) {
                        telemetry.addData("seen", "%d (%s)", d.id, d.metadata != null ? d.metadata.name : "no-meta");
                        if (d.metadata != null && isDesiredTag(d.id)) {
                            seenTagId = d.id;
                            telemetry.addData("chosen", seenTagId);
                            telemetry.update();
                            break;
                        }
                    }
                    telemetry.update();
                    sleep(60);
                }
            }

            if (seenTagId >= 0 && isDesiredTag(seenTagId)) {
                telemetry.addData("Tag to align to", seenTagId);
                telemetry.update();
                boolean success = alignToTagAndCenter(seenTagId, 6000);
                telemetry.addData("Align result", success ? "Centered" : "Timed out");
                telemetry.update();
            } else {
                telemetry.addLine("No desired tag found — ending test.");
                telemetry.update();
            }
            stopAllDrivePower();
            telemetry.addLine("Test complete.");
            telemetry.update();
        }
    }

    // -----------------------
    // VISION helpers
    // -----------------------
    private void initAprilTag() {
        aprilTag = new AprilTagProcessor.Builder().build();
        aprilTag.setDecimation(2);

        if (USE_WEBCAM) {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessor(aprilTag)
                    .build();
        } else {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessor(aprilTag)
                    .build();
        }
    }

    private void sampleTagBeforeStart(double timeoutSeconds) {
        double start = runtime.seconds();
        while (!isStarted() && !isStopRequested() && (runtime.seconds() - start) < timeoutSeconds) {
            List<AprilTagDetection> detections = aprilTag.getDetections();
            for (AprilTagDetection det : detections) {
                telemetry.addData("preseen", "%d (%s)", det.id, det.metadata != null ? det.metadata.name : "no-meta");
                if (det.metadata != null && isDesiredTag(det.id)) {
                    seenTagId = det.id;
                    telemetry.addData("prechosen", seenTagId);
                    telemetry.update();
                    return;
                }
            }
            telemetry.update();
            sleep(50);
        }
    }

    private boolean isDesiredTag(int id) {
        for (int x : DESIRED_TAG_IDS) if (x == id) return true;
        return false;
    }
    private boolean alignToTagAndCenter(int targetId, long timeoutMs) {
        long start = System.currentTimeMillis();
        while (opModeIsActive() && (System.currentTimeMillis() - start) < timeoutMs) {

            // find the tag detection for our target ID
            List<AprilTagDetection> detections = aprilTag.getDetections();
            AprilTagDetection found = null;
            for (AprilTagDetection d : detections) {
                if (d.metadata != null && d.id == targetId && isDesiredTag(d.id)) {
                    found = d;
                    break;
                }
            }

            if (found == null) {
                // rotate slowly to search for tag
                setTankPower(0.10, -0.10);
                telemetry.addData("Searching", "for tag %d", targetId);
                telemetry.update();
                sleep(60);
                continue;
            }

            double range = found.ftcPose.range;
            double bearing = found.ftcPose.bearing;

            double rangeError = range - DESIRED_DISTANCE_INCHES;
            double headingError = bearing;

            double drive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
            double turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);

            double left = drive - turn;
            double right = drive + turn;

            double max = Math.max(Math.abs(left), Math.abs(right));
            if (max > 1.0) {
                left /= max;
                right /= max;
            }

            setTankPower(left, right);

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

    private void setTankPower(double left, double right) {
        topL.setPower(left);
        bottomL.setPower(left);
        topR.setPower(right);
        bottomR.setPower(right);
    }

    private void stopAllDrivePower() {
        setTankPower(0, 0);
    }
  
    private void setManualExposure(int exposureMS, int gain) {
        if (visionPortal == null) return;

        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting for stream");
            telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        if (!isStopRequested()) {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long) exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }
    }
}
