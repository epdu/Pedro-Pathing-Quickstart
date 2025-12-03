package org.firstinspires.ftc.teamcode.pedroPathing;//package org.firstinspires.ftc.teamcode.pedroPathing;
//import com.bylazar.configurables.annotations.Configurable;
//import com.bylazar.telemetry.PanelsTelemetry;
//import com.bylazar.telemetry.TelemetryManager;
//import com.pedropathing.follower.Follower;
//import com.pedropathing.geometry.BezierLine;
//import com.pedropathing.geometry.Pose;
//import com.pedropathing.paths.HeadingInterpolator;
//import com.pedropathing.paths.Path;
//import com.pedropathing.paths.PathChain;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//
//import java.util.function.Supplier;
//
//@Configurable
//@TeleOp(name = "A Blue Streak Qualifier")
//public class TeleOpQualifer extends OpMode {
//    HardwareQualifer robot = new HardwareQualifer();
//    private Follower follower;
//    public static Pose startingPose; //See ExampleAuto to understand how to use this
//    private boolean automatedDrive;
//    private Supplier<PathChain> pathChain;
//    private TelemetryManager telemetryM;
//    private boolean slowMode = false;
//    private double slowModeMultiplier = 0.5;
//    private double powerMultiplier = 0.9;
//    public String fieldOrRobotCentric = "robot";
//    @Override
//    public void init() {
//        robot.init(hardwareMap);
//        follower = Constants.createFollower(hardwareMap);
//        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
//        follower.update();
//        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
//
//        pathChain = () -> follower.pathBuilder() //Lazy Curve Generation
//                .addPath(new Path(new BezierLine(follower::getPose, new Pose(45, 98))))
//                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(45), 0.8))
//                .build();
//        //预定义自动路径（从当前位置到坐标(45,98)）
//    }
//
//    @Override
//    public void start() {
//        //The parameter controls whether the Follower should use break mode on the motors (using it is recommended).
//        //In order to use float mode, add .useBrakeModeInTeleOp(true); to your Drivetrain Constants in Constant.java (for Mecanum)
//        //If you don't pass anything in, it uses the default (false)
//        follower.startTeleopDrive();  // 启动手动驾驶模式
//    }
//
//    @Override
//    public void loop() {
//        //Call this once per loop
//        follower.update();
//        telemetryM.update();
//
//        if (!automatedDrive) {
//            //Make the last parameter false for field-centric
//            //In case the drivers want to use a "slowMode" you can scale the vectors
//
//            //This is the normal version to use in the TeleOp
//            if (!slowMode) follower.setTeleOpDrive(
//                    -gamepad1.left_stick_y,
//                    -gamepad1.left_stick_x,
//                    -gamepad1.right_stick_x,
//                    true // Robot Centric
//            );
//
//                //This is how it looks with slowMode on
//            else follower.setTeleOpDrive(
//                    -gamepad1.left_stick_y * slowModeMultiplier,
//                    -gamepad1.left_stick_x * slowModeMultiplier,
//                    -gamepad1.right_stick_x * slowModeMultiplier,
//                    true // Robot Centric
//            );
//        }
//
//        //Automated PathFollowing
//        if (gamepad1.aWasPressed()) {
//            follower.followPath(pathChain.get());
//            automatedDrive = true;
//        }
//
//        //Stop automated following if the follower is done
//        if (automatedDrive && (gamepad1.bWasPressed() || !follower.isBusy())) {
//            follower.startTeleopDrive();
//            automatedDrive = false;
//        }
//
//        //Slow Mode
//        if (gamepad1.rightBumperWasPressed()) {
//            slowMode = !slowMode;
//        }
//
//        //Optional way to change slow mode strength
//        if (gamepad1.xWasPressed()) {
//            slowModeMultiplier += 0.25;
//        }
//
//        //Optional way to change slow mode strength
//        if (gamepad2.yWasPressed()) {
//            slowModeMultiplier -= 0.25;
//        }
//
//        telemetryM.debug("position", follower.getPose());
//        telemetryM.debug("velocity", follower.getVelocity());
//        telemetryM.debug("automatedDrive", automatedDrive);
//    }
//
//
//
////////////////////////////////////////Methods/////////////////////////////////
//    public void moveDriveTrain_RobotCentric () {
//        double robot_y = gamepad1.left_stick_y; // Remember, Y stick value is reversed
//        double robot_x = gamepad1.left_stick_x;
//        double robot_rx = gamepad1.right_stick_x * 0.5; // If a smooth turn is required 0.5
//
//        double fl = robot_y - robot_x - robot_rx;
//        double bl = robot_y + robot_x - robot_rx;
//        double fr = robot_y + robot_x + robot_rx;
//        double br = robot_y - robot_x + robot_rx;
//
//        robot.LFMotor.setPower(fl * powerMultiplier);
//        robot.LBMotor.setPower(bl * powerMultiplier);
//        robot.RFMotor.setPower(fr * powerMultiplier);
//        robot.RBMotor.setPower(br * powerMultiplier);
//
//    }
//
//
//
//    public void moveDriveTrain_FieldCentric() {
//        double y = gamepad1.left_stick_y * (0.45); // Remember, Y stick value is reversed
//        double x = -gamepad1.left_stick_x * (0.45);
//        double rx = -gamepad1.right_stick_x * (0.45); //*(0.5) is fine
//
//        double botHeading = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
//        // Rotate the movement direction counter to the bot's rotation
//        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
//        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
//
//        rotX = rotX * 1.1;  // Counteract imperfect strafing
//
//        // Denominator is the largest motor power (absolute value) or 1
//        // This ensures all the powers maintain the same ratio,
//        // but only if at least one is out of the range [-1, 1]
//        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
//        double frontLeftPower = (rotY + rotX + rx) / denominator;
//        double backLeftPower = (rotY - rotX + rx) / denominator;
//        double frontRightPower = (rotY - rotX - rx) / denominator;
//        double backRightPower = (rotY + rotX - rx) / denominator;
//
//        robot.LFMotor.setPower(frontLeftPower * powerMultiplier);
//        robot.LBMotor.setPower(backLeftPower * powerMultiplier);
//        robot.RFMotor.setPower(frontRightPower * powerMultiplier);
//        robot.RBMotor.setPower(backRightPower * powerMultiplier);
//    }
//
//
////////////////////////////////////////Methods/////////////////////////////////
//}