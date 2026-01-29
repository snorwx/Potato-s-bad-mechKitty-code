package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "TeleOp Field Centric", group = "Drive")
public class TeleOpFieldCentric extends LinearOpMode {

    private final RobotHardware robot = new RobotHardware();
    private boolean fieldCentric = true;
    private boolean lastToggle = false;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        telemetry.addLine("Field Centric Drive — Press START");
        telemetry.addLine("[BACK] Toggle field/robot centric");
        telemetry.addLine("[OPTIONS] Reset heading");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Toggle field centric mode
            boolean togglePressed = gamepad1.back;
            if (togglePressed && !lastToggle) fieldCentric = !fieldCentric;
            lastToggle = togglePressed;

            // Get joystick inputs
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            // Reset heading on OPTIONS button
            if (gamepad1.options) robot.resetHeading();

            // Drive
            if (fieldCentric) {
                double botHeading = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
                double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
                double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
                rotX *= 1.1; // Counteract imperfect strafing
                robot.drive(rotY, rotX, rx);
            } else {
                robot.drive(y, x, rx);
            }

            // Telemetry
            telemetry.addData("Mode", fieldCentric ? "FIELD-CENTRIC" : "ROBOT-CENTRIC");
            telemetry.addData("Heading", "%.1f°", robot.getHeading());
            telemetry.addData("FL | FR", "%.2f | %.2f", robot.frontLeft.getPower(), robot.frontRight.getPower());
            telemetry.addData("BL | BR", "%.2f | %.2f", robot.backLeft.getPower(), robot.backRight.getPower());
            telemetry.addLine("\n[BACK] Toggle mode | [OPTIONS] Reset heading");
            telemetry.update();
            
            idle();
        }
    }
}
