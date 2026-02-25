//package org.firstinspires.ftc.teamcode.pedroPathing;
//
////                 STRO — Yesterday at 3:14 PM
////                 I am having many difficulties with programming my turret using a CRServo with a through-bore encoder. My turret is imprecise and very slow. I will send my code.
////                 package config.Subsystems;
//
//import com.qualcomm.robotcore.hardware.CRServo;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.util.Range;
//
//public class ServoTurret {
//
//    private final CRServo turretServo;
//    private final DcMotorEx encoderPort;
//
//    public double TICKS_PER_TURRET_REV = 40726.0 ;
//
//    public boolean INVERT_SERVO = false;
//    public boolean INVERT_ENCODER = false;
//
//    public double MIN_ANGLE_RAD = Math.toRadians(-80.0);
//    public double MAX_ANGLE_RAD = Math.toRadians( 80.0);
//
//    public double kP = 4.0;
//    public double kS = 0.06;
//    public double kP_SLOW = 1.8;
//
//    public double SLOW_ZONE_RAD = Math.toRadians(10.0);
//    public double MAX_POWER = 0.8;
//    public double TOLERANCE_RAD = Math.toRadians(3);
//
//    private int zeroTick = 0;
//
//    public ServoTurret(HardwareMap hw) {
//        turretServo = hw.get(CRServo.class, "turret");
//        encoderPort = hw.get(DcMotorEx.class, "FLmotor");
//
//        turretServo.setDirection(DcMotorSimple.Direction.REVERSE);
//        encoderPort.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        encoderPort.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
//    }
//
//    public void setZeroHere() {
//        zeroTick = encoderPort.getCurrentPosition();
//    }
//
//    public int getTicksFromZero() {
//        int t = encoderPort.getCurrentPosition() - zeroTick;
//        return INVERT_ENCODER ? -t : t;
//    }
//
//    public double getTurretRad0To2Pi() {
//        double rad = ticksToRad(getTicksFromZero());
//        return wrapRad2Pi(rad);
//    }
//
//    public double getTurretRadContinuous() {
//        return ticksToRad(getTicksFromZero());
//    }
//
//    public void holdAngleDeg(double deg) {
//        updateToTurretRelAngleRad(Math.toRadians(deg));
//    }
//
//    public void updateToTurretRelAngleRad(double targetRelRad) {
//        double target = Range.clip(targetRelRad, MIN_ANGLE_RAD, MAX_ANGLE_RAD);
//        double current = getTurretRadContinuous();
//        double err = target - current;
//
//        if ((current >= MAX_ANGLE_RAD && err > 0) || (current <= MIN_ANGLE_RAD && err < 0)) {
//            turretServo.setPower(0.0);
//            return;
//        }
//
//        if (Math.abs(err) <= TOLERANCE_RAD) {
//            double hold = Math.signum(err) * Math.min(kS, 0.06);
//            turretServo.setPower(hold);
//            return;
//        }
//
//        double absErr = Math.abs(err);
//        double kpUse = (absErr > SLOW_ZONE_RAD) ? kP : kP_SLOW;
//
//        double power = kpUse * err + Math.signum(err) * kS;
//        power = Range.clip(power, -MAX_POWER, MAX_POWER);
//
//        if (INVERT_SERVO) power = -power;
//        turretServo.setPower(power);
//    }
//
//    public void aimToObject(double robotX, double robotY,
//                            double objectX, double objectY,
//                            double robotHeadingRad) {
//
//        double destinationWorldRad = Math.atan2(objectY - robotY, objectX - robotX);
//        double targetRel = destinationWorldRad - robotHeadingRad;
//
//        targetRel = wrapRadPi(targetRel);
//
//        updateToTurretRelAngleRad(targetRel);
//    }
//    public double ticksToRad(int ticks) {
//... (19 lines left)
//
//package config.Subsystems;.txt
//        4 KB
//        Phelps — Yesterday at 3:46 PM
//        Guys i use the pedro pathing field centric drive but i have a problem wich the robot dont turn and walk in a same time , how i fix this ?
//                Phelps — Yesterday at 3:48 PM
//        @Android Studio Help
//        타코 고양이 |22161| — Yesterday at 3:48 PM
//        @Programming Help
//        harryivth — Yesterday at 3:54 PM
//        Omg I have done this so many times that I just wrote it as a comment at the top of my action files
//        Aayan 31418 hayden is freeeeeeee
//
// — Yesterday at 3:56 PM
//        just use the gm0 code
//        !gm0
//        Dozer 2
//        APP
// — Yesterday at 3:56 PM
//        https://gm0.org/
//        Aayan 31418 hayden is freeeeeeee
//
// — Yesterday at 3:56 PM
//        just look up field centric
//        MrFishy | 7247
//
// — Yesterday at 3:59 PM
//        W cotl pfp
//                Zax71
//
// — Yesterday at 4:50 PM
//        You can just click on the “extends run” when you hover over your run method
//        rain — Yesterday at 4:51 PM
//        does calling .build().run(telemetryPacket) immediately after defining a roadrunner TrajectoryActionBuilder work? Android Studio seems to have no issues with it
//        Zach | Volunteer™
//
// — Yesterday at 4:51 PM
//        you have to call Actions.runBlocking on it
//                Heinoushare
//
// — Yesterday at 4:51 PM
//        Do color sensors remember their set gain in new opmodes/after a robot restart?
//                rain — Yesterday at 4:52 PM
//        It only finishes blocking when you actually reach the position
//        Right?
//                Zach | Volunteer™
//
// — Yesterday at 4:53 PM
//                yes
//        calling run on the trajectory action object will run a single iteration
//        so it will start the timer and then set drive powers to wherever the robot should be at t=0
//        Rithek | 15455
//
// — Yesterday at 5:40 PM
//        I’m trying to use the neural network classifier on the limelight 3a, but uploading a tensorflow lite model and the label file from teachable machine doesn’t seem to change the labels shown on the screen at all. Does the limelight nn classifier work?
//        Elijah| Lightsaders | 12928 — Yesterday at 6:05 PM
//        Is there any sample code for using the rev encoder and octoquad in absolute mode for precise values between 360 and 0 degrees
//        tabasco — Yesterday at 6:06 PM
//        Im using pedro but rotating the bot makes it go into a circle. I feel like i’ve checked everything: offsets, directions
//        using pinpoint btw
//        rek lil minion | 12599 #FREEREW
//
// — Yesterday at 6:11 PM
//        Are you using radians or degrees for your headings
//        It should be radians, you can use Math.toRadians(degree amt)
//        tabasco — Yesterday at 6:13 PM
//        I meant in the localization test, do you think it’s a config thing?
//        rek lil minion | 12599 #FREEREW
//
// — Yesterday at 6:13 PM
//                Oh
//        I do not know abt that then, you might want to ask in pedro server
//        https://discord.gg/D9cfyDW48
//        Pedro Pathing
//        Pedro Pathing
//        371 Online
//        3,459 Members
//        Est. Jul 2024
//        The Official Pedro Pathing Discord - Support, Learn, and Share about Pedro Pathing - An Advanced FTC Path Follower.
//
//        Go to Server
//        tabasco — Yesterday at 6:14 PM
//        It’s supposed to be plugged in to the i2c port right?
//                Tarun 24799 12791aRole icon, Worlds Inspire Winner — Yesterday at 6:14 PM
//        it’s likely you’re offsets (they might need to be flipped negative/positive) or encoder directions
//        that’s usually always the issue
//        Tarun 24799 12791aRole icon, Worlds Inspire Winner — Yesterday at 6:14 PM
//                yes
//        tabasco — Yesterday at 6:14 PM
//        I used the same setup and offsets for 3 wheel and it worked fine.
//                Tarun 24799 12791aRole icon, Worlds Inspire Winner — Yesterday at 6:15 PM
//        have you tried making the offsets negative
//        and are you sure the offsets are updating correctly
//        tabasco — Yesterday at 6:15 PM
//        ok I can try that. forward and strafe were working fine
//        타코 고양이 |22161| — Yesterday at 6:15 PM
//        Is the boolean also set to false
//        Tarun 24799 12791aRole icon, Worlds Inspire Winner — Yesterday at 6:15 PM
//        forward and strafe are only affected by direction, not offset
//        타코 고양이 |22161| — Yesterday at 6:15 PM
//        Wait wrong person
//        tabasco — Yesterday at 6:16 PM
//        my strafe pod is def right. would the forward pod being flipped cause the issue?
//                Tarun 24799 12791aRole icon, Worlds Inspire Winner — Yesterday at 6:17 PM
//        again, forward and strafe movement are not at all impacted by offsets. offsets only affect rotation
//        if the bot isn’t rotating in place on dashboard when it is in real life, try flipping both the strafe and forward offset (try all 4 combinations)
//        tabasco — Yesterday at 6:18 PM
//        Ok thank you! I’ll try tmr when I see the bot again lol
//        mathz #16052
//
// — Yesterday at 6:33 PM
//        what  kind of pidf controller you guys recommend for flywheel?
//                Spaceman|6133|The Nuts! Goof
//
// — Yesterday at 6:34 PM
//        what is max tps?
//        Elijah| Lightsaders | 12928 — Yesterday at 6:41 PM
//        How do I configure the ocotquad for pwm mode
//        michael 4345
//
// — Yesterday at 6:49 PM
//        Also make sure the offsets are from the center of rotation of ur bot, not the actual center
//        inade | 21227 — Yesterday at 6:51 PM
//        @Limelight Help is there some reason why the limelight mt1 pose is always offset weirdly? Ik the variability is high but why is it not oscillating around the correct pose? Our offsets are double checked
//        tabasco — Yesterday at 6:54 PM
//        Ooo does that mean like if an intake is sticking out of the drive train not the center of the bot ?
//                michael 4345
//
// — Yesterday at 6:55 PM
//        just spin ur robot and find where it rotates around, usually in middle of drivetrain
//        who is this guy 🤨
//
// — Yesterday at 7:00 PM
//        channel bank config
//        luca 20381
//
// — Yesterday at 7:08 PM
//        make sure the constants you set in the web app are correct
//        if your ll is at an angle or anything
//        inade | 21227 — Yesterday at 7:09 PM
//        this stuff is all correct but is there anything else we need to tune?
//        Image
//        Cactus2456 — Yesterday at 7:12 PM
//        I'm having an issue with aim in the far zone.
//
//        When the robot is pointing +X, the aim is off by a few degrees to the right. When its pointing -X, the aim is off by a few degrees to left.
//package org.firstinspires.ftc.teamcode.Robot.commands;
//
//import com.arcrobotics.ftclib.command.CommandBase;
//
//import org.firstinspires.ftc.teamcode.PP.FieldConstants;
//import org.firstinspires.ftc.teamcode.Robot.subsystems.Drive;
//
//        PPTracking.java
//        3 KB
//        luca 20381
//
// — Yesterday at 7:13 PM
//                idts
//        if it’s always a constant offset you can just add that when you set your position
//        but i doubt it’s content
//        constant
//        UnheardWolf344 |  30380
//
// — Yesterday at 7:19 PM
//        are you accounting for the turret being not in the center of the bot
//        wait are you using a turret
//        inade | 21227 — Yesterday at 7:19 PM
//        Yea it lwk doesn't seem constant, especially with large angle relative to goal, is there anything else we can do to improve accuracy?
//        Cactus2456 — Yesterday at 7:19 PM
//                yeah
//        B2Body — Yesterday at 7:26 PM
//        Has anyone had an issue with their servos losing power during tele and not holding their positions. It either goes to the position delayed or just can move freely then snaps back to it after a bit. Using an axon max
//        macProblems | NULL |
//
// — Yesterday at 7:33 PM
//        That usually happens when the axon is under too much stress and gives up to avoid damage
//﻿
//        Elijah| Lightsaders | 12928 — Yesterday at 6:05 PM
//        Is there any sample code for using the rev encoder and octoquad in absolute mode for precise values between 360 and 0 degrees
//        penguinatorrs — 2/23/2026 7:27 PM
//        Does pinpoint work with rev encoders
//        sebi | 15996 — 2/20/2026 1:14 AM
//        Has anyone had problems using a 3D-printed case for the REV Through Bore Encoder?
//                For example, wrong readings, random values, or unstable signals?
//        I saw on the official website that the encoder should not be taken apart, so I’m not sure if changing the case or using a custom 3D-printed one can cause problems.
//        Rajat Captain Of 11997 & 20712M
//
// — 2/19/2026 11:41 PM
//        So like I checked everything and only thing I can think of for a shortage is the rev encoder wires that I crimped
//        Rajat Captain Of 11997 & 20712M
//
// — 2/19/2026 11:38 PM
//        Do you think crimping from the rev encoders could cook me?
//                Dog
//
//        OP
// — 2/18/2026 9:00 PM
//        We use rev encoder for the rotation but we use axon encoder to set the position during auto
//        Rajat Captain Of 11997 & 20712M
//
// — 2/17/2026 9:31 PM
//        Can you add a rev encoder?
//        Mosely | 5627 | Captain
//
//        OP
// — 2/16/2026 11:36 PM
//        We are implementing a turret into our robot, and we currently drive rotation with gears off of a 312 rpm gobilda motor. (Yes I know its slow, but we will speed up at some point). However, if our turret gets caught it can skip the gears. We already plan on using a rev through bore encoder for our rotation tracking separate of our motor, but could a hall effect sensor work if we set it up at absolute zero? Just as a extra precaution?
//        Speedy 20265
//
// — 2/16/2026 3:14 PM
//        you cant use a rev encoder as absolute
//        alex19458
//
// — 2/16/2026 12:10 AM
//        but yea it was mainly bc the servo shaft and drive shaft are belted, so the rev encoder is jst in case the belt skips, the pod can still reach the desired location. (although im not sure if thats an issue since i havent built it)
//        Rares2787<21031> — 2/15/2026 12:31 PM
//        does the control hub support the ELC encoder V2 on absolute mode?(or the rev trough bore encoder?
//                Iuli | 20265
//
// — 2/13/2026 8:54 AM
//        Do you guys know how do I program the rev encoder with octoquad?
//                Pengu | Ishaan | 20240 Slingshot
//
// — 2/13/2026 1:16 AM
//        can you plug in rev through bore encoder into analog device port to recieve data from it?
//                WolfBuster101
//
// — 2/12/2026 9:33 PM
//        Has anyone gotten issues with the rev through bore encoder v2 not outputting the correct velocity? @Programming Help
//        Arush | 23511 | SolversLib
//
// — 2/12/2026 1:40 PM
//        did not even know there was a v2 rev encoder
//        arhaan 20240
//
// — 2/12/2026 1:37 PM
//        @Arush | 23511 | SolversLib have yall had any problems with v1 rev encoder for turret
//        arhaan 20240
//
// — 2/12/2026 10:38 AM
//        how much of a difference will the v2 have over the v1 rev bore encoder for a turret?
//        S-T-F || FTC 12560
//        OP
// — 2/6/2026 5:06 PM
//        Does anyone have any recommendations for color sensors and external encoders?
//
//                We are not satisfied with rev color sensor v3 and we need am encoder that also works at high speeds (for example an intake) since the rev through bore encoder is just not made for that
//        S-T-F || FTC 12560 — 2/6/2026 4:52 PM
//        All cable management inside 2 walls and 80% free space robot after a whole day. Now i am ready to go to sleep for the next 2 seasons 💀
//
//‼️Also does anyone have any recommendations for color sensors and external encoders? Rn we are using rev color sensor v3 and rev through bore encoder but we are looking for something better
//        Teaa
//
//                OP
// — 2/5/2026 11:15 AM
//        hi, i have a turret on an axon max and a rev through bore encoder that i want to aim at the goals using mainly pinpoint corrected by limelight. i calibrated the PID on the turret and converted the encoder reading and everything, but when i test it, after a while it seems to behave as though the aiming target shifts and aims too far right or even backwards. can someone take a look at my code and help me improve it?
//
//                the axon's angle values are negative to the right and positive to the left, with a -180 to 180 range and a 1:1 gear ratio
//
//        pls help
////=====TURRET CLASS======
//
//package org.firstinspires.ftc.teamcode.programs.subsystems;
//
//import com.arcrobotics.ftclib.command.SubsystemBase;
//import com.qualcomm.hardware.limelightvision.LLResultTypes;
//
//        code.txt
//        12 KB
//        ben | FTC #24152 Apollo — 2/3/2026 2:26 PM
//        anyone know why our robot shuts down when using the rev through bore encoder v2? it happens when we run our flywheel (it's attached to the flywheel)
//        Caleb 13201
//
// — 2/1/2026 11:31 AM
//        there’s the rev encoder or the elc one, idk much about em tho
//        ben | FTC #24152 Apollo — 2/1/2026 2:05 AM
//        Does anyone know where the rev through bore encoder is supposed to go in terms of wiring it to the control hub? I’ve heard that it should go in the motor encoder ports, and I’ve heard that it should plug in to the analog ports; which one is correct?
//        Max Schwehr
//
// — 1/31/2026 11:07 PM
//        Do you guys know what type of thing I select in the configuration menu on the driver hub for a REV Through Bore Encoder?
//                ARG 26394 Mecha Minds — 1/31/2026 10:12 PM
//        I can't use rev bore encoder bc I don't have a long enough wire so I have to use the encoder for both motors
//package config.Subsystems;
//
//import com.qualcomm.robotcore.hardware.CRServo;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.util.Range;
//
//        public class ServoTurret {
//
//            private final CRServo turretServo;
//            private final DcMotorEx encoderPort;
//
//            public double TICKS_PER_TURRET_REV = 40726.0 ;
//
//            public boolean INVERT_SERVO = false;
//            public boolean INVERT_ENCODER = false;
//
//            public double MIN_ANGLE_RAD = Math.toRadians(-80.0);
//            public double MAX_ANGLE_RAD = Math.toRadians( 80.0);
//
//            public double kP = 4.0;
//            public double kS = 0.06;
//            public double kP_SLOW = 1.8;
//
//            public double SLOW_ZONE_RAD = Math.toRadians(10.0);
//            public double MAX_POWER = 0.8;
//            public double TOLERANCE_RAD = Math.toRadians(3);
//
//            private int zeroTick = 0;
//
//            public ServoTurret(HardwareMap hw) {
//                turretServo = hw.get(CRServo.class, "turret");
//                encoderPort = hw.get(DcMotorEx.class, "FLmotor");
//
//                turretServo.setDirection(DcMotorSimple.Direction.REVERSE);
//                encoderPort.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//                encoderPort.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
//            }
//
//            public void setZeroHere() {
//                zeroTick = encoderPort.getCurrentPosition();
//            }
//
//            public int getTicksFromZero() {
//                int t = encoderPort.getCurrentPosition() - zeroTick;
//                return INVERT_ENCODER ? -t : t;
//            }
//
//            public double getTurretRad0To2Pi() {
//                double rad = ticksToRad(getTicksFromZero());
//                return wrapRad2Pi(rad);
//            }
//
//            public double getTurretRadContinuous() {
//                return ticksToRad(getTicksFromZero());
//            }
//
//            public void holdAngleDeg(double deg) {
//                updateToTurretRelAngleRad(Math.toRadians(deg));
//            }
//
//            public void updateToTurretRelAngleRad(double targetRelRad) {
//                double target = Range.clip(targetRelRad, MIN_ANGLE_RAD, MAX_ANGLE_RAD);
//                double current = getTurretRadContinuous();
//                double err = target - current;
//
//                if ((current >= MAX_ANGLE_RAD && err > 0) || (current <= MIN_ANGLE_RAD && err < 0)) {
//                    turretServo.setPower(0.0);
//                    return;
//                }
//
//                if (Math.abs(err) <= TOLERANCE_RAD) {
//                    double hold = Math.signum(err) * Math.min(kS, 0.06);
//                    turretServo.setPower(hold);
//                    return;
//                }
//
//                double absErr = Math.abs(err);
//                double kpUse = (absErr > SLOW_ZONE_RAD) ? kP : kP_SLOW;
//
//                double power = kpUse * err + Math.signum(err) * kS;
//                power = Range.clip(power, -MAX_POWER, MAX_POWER);
//
//                if (INVERT_SERVO) power = -power;
//                turretServo.setPower(power);
//            }
//
//            public void aimToObject(double robotX, double robotY,
//                                    double objectX, double objectY,
//                                    double robotHeadingRad) {
//
//                double destinationWorldRad = Math.atan2(objectY - robotY, objectX - robotX);
//                double targetRel = destinationWorldRad - robotHeadingRad;
//
//                targetRel = wrapRadPi(targetRel);
//
//                updateToTurretRelAngleRad(targetRel);
//            }
//            public double ticksToRad(int ticks) {
//                return (ticks / TICKS_PER_TURRET_REV) * 2.0 * Math.PI;
//            }
//
//            public int radToTicks(double rad) {
//                return (int) Math.round((rad / (2.0 * Math.PI)) * TICKS_PER_TURRET_REV);
//            }
//
//            public static double wrapRadPi(double a) {
//                while (a > Math.PI) a -= 2.0 * Math.PI;
//                while (a < -Math.PI) a += 2.0 * Math.PI;
//                return a;
//            }
//
//            public static double wrapRad2Pi(double a) {
//                a %= (2.0 * Math.PI);
//                if (a < 0) a += 2.0 * Math.PI;
//                return a;
//            }
//        }
//package config.Subsystems;.txt
//        4 KB