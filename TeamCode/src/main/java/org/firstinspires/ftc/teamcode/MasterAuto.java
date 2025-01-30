package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.roadrunnertuning.drive.SampleMecanumDrive;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.roadrunnertuning.trajectorysequence.TrajectorySequence;


import com.qualcomm.robotcore.hardware.*;

@Autonomous(group = "!auto")
public class MasterAuto extends LinearOpMode {
    DcMotorEx arm,lift,extender;
    Servo claw;

    boolean right;

    @Override
    public void runOpMode() throws InterruptedException {
        arm = (DcMotorEx) hardwareMap.dcMotor.get("extender");
        arm.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        lift = (DcMotorEx) hardwareMap.dcMotor.get("lift");
        lift.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        extender = (DcMotorEx) hardwareMap.dcMotor.get("arm");
        extender.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        extender.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        lift.setDirection(DcMotorSimple.Direction.REVERSE);
        arm.setDirection(DcMotorSimple.Direction.REVERSE);

        claw = hardwareMap.servo.get("claw");
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d rightPos = new Pose2d(-12,62,Math.toRadians(-90));
        Pose2d leftPos = new Pose2d(36,62,Math.toRadians(-90));
        Pose2d startPose;

        while(!isStarted()) {
            if(gamepad1.dpad_right) right = true;
            if(gamepad1.dpad_left) right = false;


            closeClaw();

            telemetry.addData("Starting Position (right: right d-pad, left: left d-pad",right ? "right" : "left");
            telemetry.update();
        }
        waitForStart();

        if(!isStopRequested()) {

            startPose = right ? rightPos : leftPos;
            drive.setPoseEstimate(startPose);

            TrajectorySequence leftTraj = drive.trajectorySequenceBuilder(startPose)
                    // Pre-load
                    .lineTo(new Vector2d(10, 42))
                    .UNSTABLE_addTemporalMarkerOffset(-.5, () -> {
                        chamber(10000);
                    }) // Arm up, lift up
                    .forward(7)
                    .UNSTABLE_addTemporalMarkerOffset(0.0, ()->{openClaw();}) // Open claw
                    .waitSeconds(0.3)

                    // First cycle
                    .back(4)
                    .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {
                        floor(10000);
                    }) // Arm down, lift down
                    .lineToLinearHeading(new Pose2d(47.5, 50, Math.toRadians(-90)))
                    .forward(3)
                    .UNSTABLE_addTemporalMarkerOffset(0.0, ()->{
                        closeClaw();
                    }) // Claw close
                    .waitSeconds(0.3)
                    .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {
                        basket(10000);
                    }) // Arm up, lift up
                    .lineToLinearHeading(new Pose2d(53, 50, Math.toRadians(45)))
                    .forward(4)
                    .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {
                        openClaw();
                    }) // Claw open
                    .waitSeconds(0.3)

                    // Second cycle
                    .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {
                        floor(10000);
                    }) // Arm down, lift down
                    .lineToLinearHeading(new Pose2d(57.5, 50, Math.toRadians(-90)))
                    .forward(3)
                    .UNSTABLE_addTemporalMarkerOffset(0.0, ()->{
                        closeClaw();
                    }) // Claw close
                    .waitSeconds(0.3)
                    .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {
                        basket(10000);
                    }) // Arm up, lift up
                    .lineToLinearHeading(new Pose2d(53, 50, Math.toRadians(45)))
                    .forward(4)
                    .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {
                        openClaw();
                    }) // Claw open
                    .waitSeconds(0.3)

                    .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {closeClaw();})
                    .waitSeconds(0.3)

                    // Park
                    .lineToLinearHeading(new Pose2d(34, 12, Math.toRadians(180)))
                    .forward(10)
                    .UNSTABLE_addTemporalMarkerOffset(-1.0, () -> {
                        arm(3600, 5000);
                    })
                    .waitSeconds(3)
                    .build();

            TrajectorySequence rightTraj = drive.trajectorySequenceBuilder(startPose)
                    // Pre-load
                    .addTemporalMarker(-0.6, () -> {
                        chamber(10000);
                    }) // Arm up, lift up
                    .splineToConstantHeading(new Vector2d(-10, 35), Math.toRadians(257.62))
                    .UNSTABLE_addTemporalMarkerOffset(0.0, ()->{openClaw();}) // Open claw
                    .waitSeconds(0.3)

                    // To first
                    .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {
                        wall(10000);
                    }) // lift down, arm reset
                    .back(4)
                    .splineTo(new Vector2d(-34.46, 29.30), Math.toRadians(268.49))
                    .splineToSplineHeading(new Pose2d(-48.76, 9.04, Math.toRadians(90.00)), Math.toRadians(180.00))
                    .lineToConstantHeading(new Vector2d(-48, 51)) // Push first

                    // Grab from wall
                    .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {closeClaw();})
                    .waitSeconds(0.3)
                    .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {lift(500, 10000);})
                    .waitSeconds(0.3)
                    .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {chamber(10000);})


                    .lineToLinearHeading(new Pose2d(-8.84, 40.65, Math.toRadians(-90.00)))
                    .forward(3)
                    .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {openClaw();})
                    .waitSeconds(0.3)

                    // To second
                    .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {
                        wall(10000);
                    }) // lift down, arm reset
                    .back(4)
                    .lineToLinearHeading(new Pose2d(-48, 46, Math.toRadians(90.00)))
                    .forward(5)

                    // Grab from wall
                    .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {closeClaw();})
                    .waitSeconds(0.3)
                    .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {lift(500, 10000);})
                    .waitSeconds(0.3)
                    .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {chamber(10000);})
                    .lineToLinearHeading(new Pose2d(-8.84, 40.65, Math.toRadians(-90.00)))
                    .forward(3)
                    .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {openClaw();})
                    .waitSeconds(0.3)
                    .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {reset(10000);})
                    .back(10)
                    .lineTo(new Vector2d(-43, 50))
                    .build();

            drive.followTrajectorySequence(right ? rightTraj : leftTraj);

            telemetry.addData("Starting Position",right ? "right" : "left");
            telemetry.update();
        }
    }

    public void arm(int pos, double velocity) {
        arm.setTargetPosition(pos);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setVelocity(velocity);

        telemetry.addData("Moving Arm to",pos);
        telemetry.update();
    }
    public void wall(double velocity) {arm(7000, velocity);
        lift(0, velocity);
    }

    public void basket(double velocity) {arm(4023, velocity);
        lift(6000, velocity);}

    public void floor(double velocity) {arm(10000, velocity); lift(0, velocity);}
    public void chamber(double velocity) {arm(4500, velocity);
        lift(545, velocity);}
    public void reset(double velocity) {arm(0, velocity);
        lift(0, velocity);}

    public void lift(int pos, double velocity) {
        lift.setTargetPosition(pos);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setVelocity(velocity);

        telemetry.addData("Lifting to",pos);
        telemetry.update();
    }

    public void openClaw() {
        claw.setPosition(0.4);
    }

    public void closeClaw() {
        claw.setPosition(0.8);
    }



}