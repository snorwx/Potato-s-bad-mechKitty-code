package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TeleOp Main", group = "Linear Opmode")
public class TeleOpMain extends LinearOpMode {

    private RobotHardware robot = new RobotHardware();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Drive control
            double axial   = -gamepad1.left_stick_y;
            double lateral =  gamepad1.left_stick_x;
            double yaw     =  gamepad1.right_stick_x;

            robot.drive(axial, lateral, yaw);

            // Reset heading on OPTIONS
            if (gamepad1.options) robot.resetHeading();

            // Telemetry
            telemetry.addData("Heading", "%.1f°", robot.getHeading());
            telemetry.addData("FL | FR", "%.2f | %.2f", robot.frontLeft.getPower(), robot.frontRight.getPower());
            telemetry.addData("BL | BR", "%.2f | %.2f", robot.backLeft.getPower(), robot.backRight.getPower());
            telemetry.addLine("\n[OPTIONS] Reset heading");
            telemetry.update();
            
            idle();
        }
    }
}