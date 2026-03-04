//package org.firstinspires.ftc.teamcode.pedroPathing;
//
////=====TURRET CLASS======
//
////package org.firstinspires.ftc.teamcode.programs.subsystems;
//
//import com.arcrobotics.ftclib.command.SubsystemBase;
//import com.qualcomm.hardware.limelightvision.LLResultTypes;
//import com.qualcomm.hardware.limelightvision.Limelight3A;
//import com.qualcomm.hardware.limelightvision.LLResult;
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
//import org.firstinspires.ftc.teamcode.programs.utils.RTPAxon;
//import org.firstinspires.ftc.teamcode.programs.utils.Robot;
//
//public class TurretCR extends SubsystemBase {
//
//    private final Robot robot = Robot.getInstance();
//    private final Limelight3A limelight;
//    private RTPAxon axon;
//
//    public static double targetAngle = 0.0;
//    public static double kP = 0.00545;
//    public static double kI = 0.0000001;
//    public static double kD = 0.0000000005;
//    public static double MAX_ANGLE = 180.0;
//
//    public TurretCR() {
//        this.limelight = robot.limelight;
//        this.axon = robot.axon;
//    }
//
//    public void initialize(){
//        axon.initialize();
//    }
//
//    private void updateHeadings(int targetID, boolean targetRedGoal) {
//        LLResult ll = limelight.getLatestResult();
//        boolean seesTargetID = false;
//
//        if (ll != null && ll.isValid()) {
//            for (LLResultTypes.FiducialResult apriltag : ll.getFiducialResults()) {
//                if (apriltag.getFiducialId() == targetID) {
//                    seesTargetID = true;
//                    break;
//                }
//            }
//        }
//
//        if (ll != null && ll.isValid() && seesTargetID) {
//            // tx and axon current angle are both degrees
//            targetAngle = axon.getCurrentAngle() - ll.getTx();
//        }
//        else if (robot.pinpoint != null) {
//            robot.pinpoint.update();
//            Pose2D pose = robot.pinpoint.getPosition();
//
//            double robotX = pose.getX(DistanceUnit.INCH);
//            double robotY = pose.getY(DistanceUnit.INCH);
//            double robotHeading = pose.getHeading(AngleUnit.RADIANS);
//
//            double goalX = targetRedGoal ? 144 : 0;
//            double goalY = 144;
//
//            double angleToGoalField = Math.atan2(goalY - robotY, goalX - robotX);
//            double relativeAngleRad = AngleUnit.normalizeRadians(angleToGoalField - robotHeading);
//
//            // Calculate the angle in degrees first
//            double calculatedAngle = Math.toDegrees(relativeAngleRad) - 90;
//
//            // Your preferred if-else clamping logic in DEGREES
//            if (calculatedAngle > MAX_ANGLE) {
//                targetAngle = MAX_ANGLE;
//            } else if (calculatedAngle < -MAX_ANGLE) {
//                targetAngle = -MAX_ANGLE;
//            } else {
//                targetAngle = calculatedAngle;
//            }
//        }
//        else {
//            targetAngle = 0;
//        }
//
//        // Final safety check to ensure targetAngle is never NaN or out of bounds
//        if (targetAngle > MAX_ANGLE) targetAngle = MAX_ANGLE;
//        if (targetAngle < -MAX_ANGLE) targetAngle = -MAX_ANGLE;
//    }
//
//    public void loop(int targetID, boolean targetRedGoal) {
//        updateHeadings(targetID, targetRedGoal);
//        axon.updatePIDCoeffs(kP, kI, kD);
//        axon.setTargetRotation(targetAngle);
//        axon.update();
//    }
//
//    public void loopAuto(double target){
//        if (target > MAX_ANGLE) targetAngle = MAX_ANGLE;
//        else if (target < -MAX_ANGLE) targetAngle = -MAX_ANGLE;
//        else targetAngle = target;
//
//        axon.updatePIDCoeffs(kP, kI, kD);
//        axon.setTargetRotation(targetAngle);
//        axon.update();
//    }
//}
//
//
//
////====== RTP AXON - AXON MAX HELPER ========
//package org.firstinspires.ftc.teamcode.programs.utils;
//
//import com.qualcomm.robotcore.hardware.CRServo;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//public class RTPAxon {
//    private final DcMotorEx encoderMotor;
//    private final CRServo servo;
//    private boolean rtp = true;
//    private double power;
//    private double maxPower = 0.4;
//    private Direction direction = Direction.FORWARD;
//    private double totalRotation;
//    private double targetRotation;
//    private final double TICKS_PER_REV = 8192.0;
//    private final double gearRatio = 1;
//
//    private double kP = 0.0;
//    private double kI = 0.0;
//    private double kD = 0.0;
//    private double integralSum = 0.0;
//    private double lastError = 0.0;
//    private double maxIntegralSum = 1.0;
//    private ElapsedTime pidTimer;
//
//    public enum Direction {
//        FORWARD,
//        REVERSE
//    }
//
//    public RTPAxon(CRServo servo, DcMotorEx encoderMotor) {
//        this.servo = servo;
//        this.encoderMotor = encoderMotor;
//        initialize(0);
//    }
//
//    public void updatePIDCoeffs(double kP, double kI, double kD) {
//        if (this.kP != kP || this.kI != kI || this.kD != kD) {
//            this.kP = kP;
//            this.kI = kI;
//            this.kD = kD;
//            resetPID();
//        }
//    }
//
//    public void initialize() {
//        initialize(0);
//    }
//
//    public void initialize(double startingAngle) {
//        servo.setPower(0);
//        this.totalRotation = startingAngle;
//        this.targetRotation = startingAngle;
//        pidTimer = new ElapsedTime();
//        pidTimer.reset();
//        resetPID();
//    }
//
//    public void setDirection(Direction direction) {
//        this.direction = direction;
//    }
//
//    public void setPower(double power) {
//        this.power = Math.max(-maxPower, Math.min(maxPower, power));
//        servo.setPower(this.power * (direction == Direction.REVERSE ? -1 : 1));
//    }
//
//    public double getPower() { return power; }
//
//    public void setMaxPower(double maxPower) { this.maxPower = maxPower; }
//
//    public void setRtp(boolean rtp) {
//        this.rtp = rtp;
//        if (rtp) resetPID();
//    }
//
//    public void setTargetRotation(double target) {
//        targetRotation = target;
//    }
//
//    public double getCurrentAngle() {
//        double encoderDegrees = (encoderMotor.getCurrentPosition() / TICKS_PER_REV) * 360.0;
//        if (direction == Direction.REVERSE) encoderDegrees *= -1;
//        return encoderDegrees / gearRatio;
//    }
//
//    public void resetPID() {
//        integralSum = 0;
//        lastError = 0;
//    }
//
//    public synchronized void update() {
//        totalRotation = getCurrentAngle();
//
//        if (!rtp) return;
//
//        double dt = pidTimer.seconds();
//        pidTimer.reset();
//
//        if (dt < 0.0001 || dt > 0.5) return;
//
//        double error = targetRotation - totalRotation;
//        integralSum += error * dt;
//        integralSum = Math.max(-maxIntegralSum, Math.min(maxIntegralSum, integralSum));
//
//        if (Math.abs(error) < 2.0) integralSum *= 0.95;
//
//        double derivative = (error - lastError) / dt;
//        lastError = error;
//
//        double output = (kP * error) + (kI * integralSum) + (kD * derivative);
//
//        if (Math.abs(error) > 0.5) {
//            setPower(output);
//        } else {
//            setPower(0);
//        }
//    }
//}
//
//
////===== TEST OPMODE =======
//
//package org.firstinspires.ftc.teamcode.programs.test;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.arcrobotics.ftclib.command.CommandOpMode;
//import com.arcrobotics.ftclib.command.CommandScheduler;
//import com.arcrobotics.ftclib.command.SequentialCommandGroup;
//import com.arcrobotics.ftclib.command.WaitCommand;
//import com.arcrobotics.ftclib.gamepad.GamepadEx;
//import com.arcrobotics.ftclib.gamepad.GamepadKeys;
//import com.qualcomm.hardware.limelightvision.LLResult;
//import com.qualcomm.hardware.limelightvision.LLResultTypes;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
//import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
//import org.firstinspires.ftc.teamcode.programs.commandbase.intake.startIntakeFront;
//import org.firstinspires.ftc.teamcode.programs.commandbase.intake.stopIntakeFront;
//import org.firstinspires.ftc.teamcode.programs.subsystems.TurretCR;
//import org.firstinspires.ftc.teamcode.programs.utils.Robot;
//import org.firstinspires.ftc.teamcode.programs.utils.geometry.PoseRR;
//@TeleOp(name = "!!NEW Pinpoint and Limelight Turret Test!!", group = "OpModes")
//public class NEWTurretLimelightAndPinpointTest extends CommandOpMode {
//    private final Robot robot = Robot.getInstance();
//    private GamepadEx gamepadEx;
//    private final FtcDashboard dashboard = FtcDashboard.getInstance();
//    double exponentialJoystickCoord_X_TURN, exponentialJoystickCoord_X_FORWARD, exponentialJoystickCoord_Y;
//    public static double constantTerm = 0.6, liniarCoefTerm = 0.7;
//
//    public double distance, ta, tx, ty, pos, x_distance, y_distance, targetAngle;
//    public Pose3D botpose;
//    public double downY = 1, upY = 0, maxDistance = 0.004, minDistance = 0.2704;
//
//    public double CAMERA_ANGLE = 18;
//    public double CAMERA_HEIGHT = 0.4;
//    @Override
//    public void initialize() {
//        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
//        CommandScheduler.getInstance().reset();
//
//        gamepadEx = new GamepadEx(gamepad1);
//        robot.initializeTurretHardware(hardwareMap);
//        robot.initializeTurret();
//
//        robot.limelight.start();
//        robot.limelight.setPollRateHz(100);
//        robot.limelight.pipelineSwitch(0);
//
//        gamepadEx.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(new SequentialCommandGroup(
//                new startIntakeFront(1),
//                new WaitCommand(1500),
//                new stopIntakeFront()
//        ));
//
//        gamepadEx.getGamepadButton(GamepadKeys.Button.Y).whenPressed(new SequentialCommandGroup(
//                new startIntakeFront(-1),
//                new WaitCommand(300),
//                new stopIntakeFront()
//        ));
//
//    }
//
//    @Override
//    public void run() {
//        CommandScheduler.getInstance().run();
//
//        exponentialJoystickCoord_X_TURN = (Math.pow(gamepad1.right_stick_x, 3) + liniarCoefTerm * gamepad1.right_stick_x) * constantTerm;
//        exponentialJoystickCoord_X_FORWARD = (Math.pow(gamepad1.left_stick_x, 3) + liniarCoefTerm * gamepad1.left_stick_x) * constantTerm;
//        exponentialJoystickCoord_Y = (Math.pow(gamepad1.left_stick_y, 3) + liniarCoefTerm * gamepad1.left_stick_y) * constantTerm;
//
//        double turnSpeed =  -exponentialJoystickCoord_X_TURN;
//        PoseRR drive = new PoseRR(-exponentialJoystickCoord_X_FORWARD, exponentialJoystickCoord_Y, turnSpeed);
//        robot.mecanum.set(drive, 0);
//
//        robot.pinpoint.update();
//
//        LLResult result = robot.limelight.getLatestResult();
//
//        robot.turret.loop(20, false);
//
//        telemetry.addData("Target Angle", TurretCR.targetAngle);
//        telemetry.update();
//    }
//}
