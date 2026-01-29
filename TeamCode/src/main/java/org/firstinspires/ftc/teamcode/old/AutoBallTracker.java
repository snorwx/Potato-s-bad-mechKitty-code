package org.firstinspires.ftc.teamcode;

import android.util.Size;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ColorSpace;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.opencv.core.Scalar;

import java.util.List;

/**
 * Autonomous Ball Tracker
 * 
 * This autonomous OpMode tracks purple or green artifacts and continuously
 * rotates the robot to keep the ball centered in the camera view.
 * 
 * Controls:
 * - The robot will automatically rotate left/right to center on detected balls
 * - Press STOP to end the OpMode
 */
@Autonomous(name = "Auto Ball Tracker", group = "Robot")
public class AutoBallTracker extends LinearOpMode {

    private RobotHardware robot = new RobotHardware();
    private ColorBlobLocatorProcessor purpleLocator;
    private ColorBlobLocatorProcessor greenLocator;
    private VisionPortal visionPortal;

    // ============================================================
    // CAMERA SETTINGS
    // ============================================================
    private static final int CAMERA_WIDTH = 640;
    private static final int CAMERA_HEIGHT = 480;
    private static final double FRAME_CENTER_X = CAMERA_WIDTH / 2.0; // 320

    // ============================================================
    // COLOR RANGES (same as TeleOpObeliskDetection)
    // ============================================================
    
    // PURPLE ARTIFACT - Wide range for varying lighting conditions
    private static final Scalar PURPLE_MIN = new Scalar(115, 50, 50);
    private static final Scalar PURPLE_MAX = new Scalar(175, 255, 255);
    
    // GREEN ARTIFACT
    private static final Scalar GREEN_MIN = new Scalar(45, 130, 70);
    private static final Scalar GREEN_MAX = new Scalar(85, 255, 255);

    // ============================================================
    // SHAPE FILTERING PARAMETERS
    // ============================================================
    private static final int MIN_BLOB_AREA = 2500;
    private static final int MAX_BLOB_AREA = 150000;
    private static final double MAX_ASPECT_RATIO = 1.6;
    private static final double MIN_DENSITY = 0.40;
    private static final double MAX_DENSITY = 0.90;
    private static final double MIN_CIRCULARITY = 0.30;

    // ============================================================
    // PID CONTROLLER SETTINGS FOR ROTATION
    // Tuned for smooth tracking without oscillation
    // ============================================================
    private static final double kP = 0.003;   // Proportional gain
    private static final double kI = 0.0001;  // Integral gain (reduces steady-state error)
    private static final double kD = 0.002;   // Derivative gain (reduces oscillation)
    
    // Deadband - if ball is within this many pixels of center, don't move
    // This prevents jittering when ball is "close enough" to centered
    private static final double CENTER_DEADBAND = 25.0;
    
    // Maximum rotation speed (0.0 to 1.0)
    private static final double MAX_TURN_SPEED = 0.35;
    
    // Minimum rotation speed (to overcome friction)
    private static final double MIN_TURN_SPEED = 0.08;

    // PID state variables
    private double integralSum = 0;
    private double lastError = 0;
    private long lastTime = 0;

    // Tracking mode: which color to track
    private enum TrackingTarget {
        PURPLE,
        GREEN,
        ANY  // Track whichever is visible (purple priority)
    }
    private TrackingTarget currentTarget = TrackingTarget.ANY;

    // Detection state
    private boolean ballDetected = false;
    private double ballX = 0;
    private double ballArea = 0;
    private String detectedColor = "NONE";

    @Override
    public void runOpMode() {
        // Initialize robot hardware
        robot.init(hardwareMap);
        
        // Initialize vision system
        initVision();

        telemetry.addLine("=== BALL TRACKER ===");
        telemetry.addLine("Robot will rotate to keep ball centered");
        telemetry.addLine("");
        telemetry.addData("Status", "Initialized - Waiting for START");
        telemetry.addData("Tracking", currentTarget.toString());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Reset PID
        resetPID();
        lastTime = System.nanoTime();

        // Main tracking loop
        while (opModeIsActive()) {
            
            // Detect artifacts
            detectBalls();
            
            // Calculate and apply rotation to center the ball
            double turnPower = 0;
            
            if (ballDetected) {
                // Calculate error: positive = ball is to the right, negative = ball is to the left
                double error = ballX - FRAME_CENTER_X;
                
                // Apply deadband - if close enough to center, don't move
                if (Math.abs(error) < CENTER_DEADBAND) {
                    turnPower = 0;
                    resetPID(); // Reset PID to prevent integral windup
                } else {
                    // Calculate PID output
                    turnPower = calculatePID(error);
                }
            } else {
                // No ball detected - stop and reset PID
                turnPower = 0;
                resetPID();
            }
            
            // Apply rotation: drive(axial, lateral, yaw)
            // axial = 0 (no forward/back)
            // lateral = 0 (no strafe)
            // yaw = turnPower (rotation only)
            robot.drive(0, 0, turnPower);

            // === TELEMETRY ===
            telemetry.addLine("=== AUTO BALL TRACKER ===");
            telemetry.addData("Mode", "TRACKING: " + currentTarget.toString());
            telemetry.addLine("");
            
            if (ballDetected) {
                telemetry.addData("TARGET", detectedColor + " BALL DETECTED");
                telemetry.addData("Position X", "%.0f / %.0f", ballX, FRAME_CENTER_X);
                telemetry.addData("Area", "%.0f px", ballArea);
                telemetry.addData("Error", "%.1f px", ballX - FRAME_CENTER_X);
                telemetry.addData("Turn Power", "%.3f", turnPower);
                
                // Visual indicator
                double error = ballX - FRAME_CENTER_X;
                if (Math.abs(error) < CENTER_DEADBAND) {
                    telemetry.addLine("✓ CENTERED!");
                } else if (error > 0) {
                    telemetry.addLine("→ Rotating RIGHT");
                } else {
                    telemetry.addLine("← Rotating LEFT");
                }
            } else {
                telemetry.addData("TARGET", "NO BALL DETECTED");
                telemetry.addLine("Searching...");
            }
            
            telemetry.addLine("");
            telemetry.addLine("Press STOP to end");
            telemetry.update();
            
            // Small delay to prevent CPU hogging
            sleep(20);
        }
        
        // Stop motors and close vision
        robot.drive(0, 0, 0);
        visionPortal.close();
    }

    /**
     * Initialize the color blob vision processors
     */
    private void initVision() {
        // Purple artifact detector
        purpleLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(new ColorRange(ColorSpace.HSV, PURPLE_MIN, PURPLE_MAX))
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.entireFrame())
                .setDrawContours(true)
                .setBlurSize(9)
                .build();
        
        // Green artifact detector
        greenLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(new ColorRange(ColorSpace.HSV, GREEN_MIN, GREEN_MAX))
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.entireFrame())
                .setDrawContours(true)
                .setBlurSize(9)
                .build();

        // Create VisionPortal
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(CAMERA_WIDTH, CAMERA_HEIGHT))
                .enableLiveView(true)
                .addProcessor(purpleLocator)
                .addProcessor(greenLocator)
                .build();
    }

    /**
     * Detect purple and green balls, select the best target
     */
    private void detectBalls() {
        ballDetected = false;
        ballX = 0;
        ballArea = 0;
        detectedColor = "NONE";

        // Find best purple blob
        double purpleX = 0, purpleArea = 0;
        boolean purpleFound = false;
        
        List<ColorBlobLocatorProcessor.Blob> purpleBlobs = purpleLocator.getBlobs();
        for (ColorBlobLocatorProcessor.Blob blob : purpleBlobs) {
            if (isValidArtifactShape(blob)) {
                int area = blob.getContourArea();
                if (area > purpleArea) {
                    purpleArea = area;
                    purpleX = blob.getBoxFit().center.x;
                    purpleFound = true;
                }
            }
        }

        // Find best green blob
        double greenX = 0, greenArea = 0;
        boolean greenFound = false;
        
        List<ColorBlobLocatorProcessor.Blob> greenBlobs = greenLocator.getBlobs();
        for (ColorBlobLocatorProcessor.Blob blob : greenBlobs) {
            if (isValidArtifactShape(blob)) {
                int area = blob.getContourArea();
                if (area > greenArea) {
                    greenArea = area;
                    greenX = blob.getBoxFit().center.x;
                    greenFound = true;
                }
            }
        }

        // Select target based on tracking mode
        switch (currentTarget) {
            case PURPLE:
                if (purpleFound) {
                    ballDetected = true;
                    ballX = purpleX;
                    ballArea = purpleArea;
                    detectedColor = "PURPLE";
                }
                break;
                
            case GREEN:
                if (greenFound) {
                    ballDetected = true;
                    ballX = greenX;
                    ballArea = greenArea;
                    detectedColor = "GREEN";
                }
                break;
                
            case ANY:
            default:
                // Track the larger/closer ball, with purple priority if similar size
                if (purpleFound && greenFound) {
                    // If purple is at least 70% the size of green, prefer purple
                    if (purpleArea >= greenArea * 0.7) {
                        ballDetected = true;
                        ballX = purpleX;
                        ballArea = purpleArea;
                        detectedColor = "PURPLE";
                    } else {
                        ballDetected = true;
                        ballX = greenX;
                        ballArea = greenArea;
                        detectedColor = "GREEN";
                    }
                } else if (purpleFound) {
                    ballDetected = true;
                    ballX = purpleX;
                    ballArea = purpleArea;
                    detectedColor = "PURPLE";
                } else if (greenFound) {
                    ballDetected = true;
                    ballX = greenX;
                    ballArea = greenArea;
                    detectedColor = "GREEN";
                }
                break;
        }
    }

    /**
     * Validate blob shape (same logic as TeleOpObeliskDetection)
     */
    private boolean isValidArtifactShape(ColorBlobLocatorProcessor.Blob blob) {
        double boxWidth = blob.getBoxFit().size.width;
        double boxHeight = blob.getBoxFit().size.height;
        int contourArea = blob.getContourArea();
        
        if (boxWidth == 0 || boxHeight == 0) return false;
        
        // Area bounds
        if (contourArea < MIN_BLOB_AREA || contourArea > MAX_BLOB_AREA) {
            return false;
        }
        
        // Aspect ratio
        double aspectRatio = Math.max(boxWidth / boxHeight, boxHeight / boxWidth);
        if (aspectRatio > MAX_ASPECT_RATIO) {
            return false;
        }
        
        // Density
        double boxArea = boxWidth * boxHeight;
        double density = contourArea / boxArea;
        if (density < MIN_DENSITY || density > MAX_DENSITY) {
            return false;
        }
        
        // Circularity
        double perimeter = blob.getContourPoints().length * 2.0;
        if (perimeter > 0) {
            double circularity = (4.0 * Math.PI * contourArea) / (perimeter * perimeter);
            if (circularity < MIN_CIRCULARITY) {
                return false;
            }
        }
        
        return true;
    }

    /**
     * Calculate PID output for smooth rotation
     * 
     * @param error Distance from center (positive = ball is right of center)
     * @return Turn power (-1.0 to 1.0, positive = turn right)
     */
    private double calculatePID(double error) {
        long currentTime = System.nanoTime();
        double dt = (currentTime - lastTime) / 1e9; // Convert to seconds
        lastTime = currentTime;
        
        // Prevent division by zero on first call
        if (dt <= 0) dt = 0.02;
        
        // Proportional term
        double pTerm = kP * error;
        
        // Integral term (accumulated error over time)
        integralSum += error * dt;
        // Anti-windup: limit integral to prevent overshoot
        integralSum = Math.max(-5000, Math.min(5000, integralSum));
        double iTerm = kI * integralSum;
        
        // Derivative term (rate of change of error)
        double derivative = (error - lastError) / dt;
        double dTerm = kD * derivative;
        lastError = error;
        
        // Calculate total output
        double output = pTerm + iTerm + dTerm;
        
        // Apply minimum power to overcome static friction
        if (Math.abs(output) > 0.001) {
            if (output > 0) {
                output = Math.max(output, MIN_TURN_SPEED);
            } else {
                output = Math.min(output, -MIN_TURN_SPEED);
            }
        }
        
        // Clamp to maximum turn speed
        output = Math.max(-MAX_TURN_SPEED, Math.min(MAX_TURN_SPEED, output));
        
        return output;
    }

    /**
     * Reset PID state (call when target is lost or centered)
     */
    private void resetPID() {
        integralSum = 0;
        lastError = 0;
    }
}
