package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class HardwareQualifier {
    HardwareMap hwMap =  null;

    public DcMotorEx RFMotor;
    public DcMotorEx LFMotor;
    public DcMotorEx RBMotor;
    public DcMotorEx LBMotor;
    public DcMotorEx MasterShooterMotorL;
    public DcMotorEx SlaveShooterMotorR;
    public Servo blocker;

    public DcMotorEx IntakeMotor;

    public DcMotorEx ShooterMotor;
    public DigitalChannel redLED ;
    public DigitalChannel greenLED;
    public VoltageSensor voltageCHub;
    public VoltageSensor voltageExHub;

    IMU imu;
    public static final double DriveTrains_POWER =  0.95 ;// reduced power of driving train motors

    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        LFMotor   = hwMap.get(DcMotorEx.class, "LFMotor");//11072025 control hub port 2
        RFMotor  = hwMap.get(DcMotorEx.class, "RFMotor"); //11072025 expansiom hub port 0
        LBMotor   = hwMap.get(DcMotorEx.class, "LBMotor");//11072025 control hub port 1
        RBMotor  = hwMap.get(DcMotorEx.class, "RBMotor"); //11072025 expansion hub port 1

        LBMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        LFMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        LFMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RFMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LBMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RBMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        LFMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RFMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LBMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RBMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



        MasterShooterMotorL = hwMap.get(DcMotorEx.class, "MasterShooterMotorL");//11072025 control  hub port 3
        SlaveShooterMotorR = hwMap.get(DcMotorEx.class, "SlaveShooterMotorR"); //11072025 expansion  hub port 2


        SlaveShooterMotorR.setDirection(DcMotorSimple.Direction.REVERSE);

        MasterShooterMotorL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        SlaveShooterMotorR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        double ShooterMotorLVelo =MasterShooterMotorL.getVelocity();
        double ShooterMotorRVelo =SlaveShooterMotorR.getVelocity();

        MasterShooterMotorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        SlaveShooterMotorR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        IntakeMotor  = hwMap.get(DcMotorEx.class, "IntakeMotor"); //11072025 control hub port 0
        IntakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        voltageCHub = hwMap.get(VoltageSensor.class, "Control Hub");
        voltageExHub = hwMap.get(VoltageSensor.class, "Expansion Hub 2");

        setAllPower(0);


        imu = hwMap.get(IMU.class, "imu");  //control I2C port 1
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        imu.initialize(parameters);

        redLED = hwMap.get(DigitalChannel.class, "red");            // digital 01
        greenLED = hwMap.get(DigitalChannel.class, "green");         // digital 01
        redLED.setMode(DigitalChannel.Mode.OUTPUT);
        greenLED.setMode(DigitalChannel.Mode.OUTPUT);

    }
    public void setMotorPower(double lF, double rF, double lB, double rB){
        LFMotor.setPower(lF*DriveTrains_POWER);
        LBMotor.setPower(lB*DriveTrains_POWER);
        RBMotor.setPower(rB*DriveTrains_POWER);
        RFMotor.setPower(rF*DriveTrains_POWER);
    }
    public void setAllPower(double p){
        setMotorPower(p,p,p,p);
    }
}
