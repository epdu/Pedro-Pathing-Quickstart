package org.firstinspires.ftc.teamcode.pedroPathing;

import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.CM;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class HardwareQualifier {
    HardwareMap hwMap =  null;

    public DcMotorEx rightFrontMotor;
    public DcMotorEx leftFrontMotor;
    public DcMotorEx rightRearMotor;
    public DcMotorEx leftRearMotor;
    public DcMotorEx MasterShooterMotorL;
    public DcMotorEx SlaveShooterMotorR;
    public Servo blocker;

    public DcMotorEx IntakeMotorL;
    public DcMotorEx IntakeMotorR;

    public DcMotorEx ShooterMotor;
    public DigitalChannel redLED ;
    public DigitalChannel greenLED;
    public VoltageSensor voltageCHub;
    public VoltageSensor voltageExHub;
    public ServoImplEx HoodArmL;
    public ServoImplEx HoodArmR;
    GoBildaPinpointDriver odo;
    IMU imu;
    public static final double DriveTrains_POWER =  0.95 ;// reduced power of driving train motors

    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        leftFrontMotor   = hwMap.get(DcMotorEx.class, "LFMotor");//11072025 control hub port 0
        leftRearMotor   = hwMap.get(DcMotorEx.class, "LBMotor");//11072025 control hub port 1
        rightFrontMotor  = hwMap.get(DcMotorEx.class, "RFMotor"); //01292026 control hub port 2
        rightRearMotor  = hwMap.get(DcMotorEx.class, "RBMotor"); //11072025 control hub port 3

        leftRearMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);


        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRearMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRearMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        MasterShooterMotorL = hwMap.get(DcMotorEx.class, "MasterShooterMotorL");//11072025 expansion  hub port 2
        SlaveShooterMotorR = hwMap.get(DcMotorEx.class, "SlaveShooterMotorR"); //11072025 expansion  hub port 3
        SlaveShooterMotorR.setDirection(DcMotorSimple.Direction.REVERSE);

        MasterShooterMotorL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        SlaveShooterMotorR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        double ShooterMotorLVelo =MasterShooterMotorL.getVelocity();
        double ShooterMotorRVelo =SlaveShooterMotorR.getVelocity();

        MasterShooterMotorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        SlaveShooterMotorR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        IntakeMotorL  = hwMap.get(DcMotorEx.class, "IntakeMotorL"); //11072025 expansion  hub port 0
        IntakeMotorL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        IntakeMotorR  = hwMap.get(DcMotorEx.class, "IntakeMotorR"); //01292026  expansion  hub port 1
        IntakeMotorR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        IntakeMotorR.setDirection(DcMotorSimple.Direction.REVERSE);


        voltageCHub = hwMap.get(VoltageSensor.class, "Control Hub");
        voltageExHub = hwMap.get(VoltageSensor.class, "Expansion Hub 2");

        setAllPower(0);

//Begin Definition and Initialization of HoodArm  and ArmR Servos

        HoodArmL = hwMap.get(ServoImplEx.class, "HoodArmL");//control hub port 2
        HoodArmR = hwMap.get(ServoImplEx.class, "HoodArmR");;//control hub port 4
        HoodArmL.setPwmRange(new PwmControl.PwmRange(500, 2500));
        HoodArmR.setPwmRange(new PwmControl.PwmRange(500, 2500));
        HoodArmR.setDirection(Servo.Direction.REVERSE);
//        initializeOArmPosition();

        ////End Definition and Initialization of outtake ArmL and ArmR Servos

        ///////////////////////////////////////GoBildaPinpointDriver//////////////////////////////
        odo = hwMap.get(GoBildaPinpointDriver.class,"odo"); //control  hub i2c port 1
//        pinpoint = hwMap.get(GoBildaPinpointDriverRR.class, "pinpoint"); // guess for RR only
         /*
        Set the odometry pod positions relative to the point that the odometry computer tracks around.
        The X pod offset refers to how far sideways from the tracking point the
        X (forward) odometry pod is. Left of the center is a positive number,
        right of center is a negative number. the Y pod offset refers to how far forwards from
        the tracking point the Y (strafe) odometry pod is. forward of center is a positive number,
        backwards is a negative number.
         */
        //  odo.setOffsets(-84.0, -224.0); //these are tuned for 3110-0002-0001 Product Insight #1
//        odo.setOffsets(23.0, -8.0);  before 0210 this is to pinpoint center
        odo.setOffsets(28.5, 156.0, CM);
        //02102025 update the off to be center of robot by suggestion for Ethan
        // robot measured with length  Y=35.8cm X=29.3cm.X pod 11.8 to the left wall
        // Y pod 2.3 cm behind the front wall
//        odo.setOffsets(-210, -150);
        /*
        Set the kind of pods used by your robot. If you're using goBILDA odometry pods, select either
        the goBILDA_SWINGARM_POD, or the goBILDA_4_BAR_POD.
        If you're using another kind of odometry pod, uncomment setEncoderResolution and input the
        number of ticks per mm of your odometry pod.
         */
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        //odo.setEncoderResolution(13.26291192);
        /*
        Set the direction that each of the two odometry pods count. The X (forward) pod should
        increase when you move the robot forward. And the Y (strafe) pod should increase when
        you move the robot to the left.
         */
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        //       odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        //REVERSED  FORWARD
        /*
        Before running the robot, recalibrate the IMU. This needs to happen when the robot is stationary
        The IMU will automatically calibrate when first powered on, but recalibrating before running
        the robot is a good idea to ensure that the calibration is "good".
        resetPosAndIMU will reset the position to 0,0,0 and also recalibrate the IMU.
        This is recommended before you run your autonomous, as a bad initial calibration can cause
        an incorrect starting value for x, y, and heading.
         */
        odo.recalibrateIMU();
        odo.resetPosAndIMU();

        ///////////////////////////////////////GoBildaPinpointDriver/////////////////////////////


        imu = hwMap.get(IMU.class, "imu");  //control I2C port 3
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        imu.initialize(parameters);
        imu.resetYaw();

        redLED = hwMap.get(DigitalChannel.class, "red");            //  Digital device  digital 0
        greenLED = hwMap.get(DigitalChannel.class, "green");         // Digital device  digital 1
        redLED.setMode(DigitalChannel.Mode.OUTPUT);
        greenLED.setMode(DigitalChannel.Mode.OUTPUT);
        redLED.setState(true);
        greenLED.setState(true);

    }
    public void setMotorPower(double lF, double rF, double lB, double rB){
        leftFrontMotor.setPower(lF*DriveTrains_POWER);
        leftRearMotor.setPower(lB*DriveTrains_POWER);
        rightRearMotor.setPower(rB*DriveTrains_POWER);
        rightFrontMotor.setPower(rF*DriveTrains_POWER);
    }
    public void setAllPower(double p){
        setMotorPower(p,p,p,p);
    }
//    public void initializeOArmPosition() {
//        if (SpecimenFourTwoPlusTwo.lastOArmLPosition > 0 || SpecimenFourTwoPlusTwo.lastOArmRPosition > 0) { // Restore saved position
//            OArmL.setPosition(SpecimenFourTwoPlusTwo.lastOArmLPosition);
//            OArmR.setPosition(SpecimenFourTwoPlusTwo.lastOArmRPosition);
//        } else { // Default position if no saved value
//            OArmL.setPosition(Constants_CS.OArmLInitializationhigher);
//            OArmR.setPosition(Constants_CS.OArmRInitializationhigher);
//        }
//    }
}
