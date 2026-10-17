package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

/**
 * TeleOp OpMode to test and calibrate Limelight 3A vision processing.
 *
 * Configured hardware name in Robot Configuration: "limelight"
 */
@TeleOp(name = "Limelight Test", group = "Test")
public class LimelightTest extends LinearOpMode {

    private Limelight3A limelight;
    private int currentPipeline = 0;
    private boolean dpadUpPressed = false;
    private boolean dpadDownPressed = false;

    @Override
    public void runOpMode() throws InterruptedException {
        // Retrieve Limelight 3A from hardware map
        try {
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
        } catch (Exception e) {
            telemetry.addData("Error", "Limelight 3A not found on hardware map! Ensure device name is 'limelight'.");
            telemetry.update();
            waitForStart();
            return;
        }

        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(currentPipeline);

        // Start polling for Limelight data
        limelight.start();

        telemetry.addData("Status", "Limelight Initialized.");
        telemetry.addData("Instructions", "Use D-Pad UP / DOWN to switch pipeline (0-9).");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Handle Pipeline switching via D-pad
            if (gamepad1.dpad_up && !dpadUpPressed) {
                currentPipeline = (currentPipeline + 1) % 10;
                limelight.pipelineSwitch(currentPipeline);
            }
            dpadUpPressed = gamepad1.dpad_up;

            if (gamepad1.dpad_down && !dpadDownPressed) {
                currentPipeline = (currentPipeline - 1 + 10) % 10;
                limelight.pipelineSwitch(currentPipeline);
            }
            dpadDownPressed = gamepad1.dpad_down;

            // Limelight Status
            LLStatus status = limelight.getStatus();
            if (status != null) {
                telemetry.addData("Limelight Name", status.getName());
                telemetry.addData("Pipeline Index", status.getPipelineIndex());
                telemetry.addData("Status", "Temp: %.1f°C | CPU: %.1f%% | FPS: %d",
                        status.getTemp(), status.getCpu(), (int) status.getFps());
            }

            // Limelight Targeting Results
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                double tx = result.getTx();
                double ty = result.getTy();
                double ta = result.getTa();

                telemetry.addData("Target Found", "YES");
                telemetry.addData("Target X (tx)", "%.2f°", tx);
                telemetry.addData("Target Y (ty)", "%.2f°", ty);
                telemetry.addData("Target Area (ta)", "%.2f%%", ta);

                // Latency
                telemetry.addData("Latency (ms)", "Cap: %.1f | Target: %.1f | Parse: %.1f",
                        result.getCaptureLatency(), result.getTargetingLatency(), result.getParseLatency());

                // 3D Botpose if AprilTag pipeline is active
                Pose3D botpose = result.getBotpose();
                if (botpose != null) {
                    telemetry.addData("Botpose 3D", "X: %.2fm, Y: %.2fm, Z: %.2fm",
                            botpose.getPosition().x,
                            botpose.getPosition().y,
                            botpose.getPosition().z);
                }

                // Fiducial / AprilTag details
                List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
                for (LLResultTypes.FiducialResult fr : fiducials) {
                    telemetry.addData("AprilTag ID", "%d (X: %.1f°, Y: %.1f°)",
                            fr.getFiducialId(), fr.getTargetXDegrees(), fr.getTargetYDegrees());
                }

                // Color Target details
                List<LLResultTypes.ColorResult> colorResults = result.getColorResults();
                for (LLResultTypes.ColorResult cr : colorResults) {
                    telemetry.addData("Color Target", "X: %.1f°, Y: %.1f°",
                            cr.getTargetXDegrees(), cr.getTargetYDegrees());
                }

                // Neural Detector details
                List<LLResultTypes.DetectorResult> detectorResults = result.getDetectorResults();
                for (LLResultTypes.DetectorResult dr : detectorResults) {
                    telemetry.addData("Detected Object", "%s (Area: %.2f%%)",
                            dr.getClassName(), dr.getTargetArea());
                }

            } else {
                telemetry.addData("Target Found", "NO (No target in view for pipeline %d)", currentPipeline);
            }

            telemetry.update();
        }

        limelight.stop();
    }
}
