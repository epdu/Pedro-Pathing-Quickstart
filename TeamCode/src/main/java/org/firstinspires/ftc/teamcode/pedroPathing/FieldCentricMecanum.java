package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class FieldCentricMecanum extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Hardware mapping
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRightMotor");

        // Set motor directions (Left side reversed)
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);

        // IMU Setup as per original code
        IMU imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));

        waitForStart();

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Reversed stick
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            // Reset Yaw/Heading if options button is pressed
            if (gamepad1.options) {
                imu.resetYaw();
            }

            // Get heading from IMU
            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Field-centric rotation math (Original logic)
            double rotX = (x * Math.cos(-botHeading) - y * Math.sin(-botHeading)) * 1.1;
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            // Calculate clipping denominator
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1.0);

            // Set the calculated power to the motors (The Fix)
            frontLeftMotor.setPower((rotY + rotX + rx) / denominator);
            backLeftMotor.setPower((rotY - rotX + rx) / denominator);
            frontRightMotor.setPower((rotY - rotX - rx) / denominator);
            backRightMotor.setPower((rotY + rotX - rx) / denominator);
        }
    }
}