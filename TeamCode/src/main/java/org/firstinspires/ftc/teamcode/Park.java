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
public class Park extends LinearOpMode {
    DcMotorEx arm,lift,extender;
    Servo claw;

    boolean wait;
    boolean right = true;
    boolean parkBool = true;

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
            if(gamepad1.x)wait = true;
            if(gamepad1.b)wait = false;
            if(gamepad1.a)parkBool = false;
            if(gamepad1.y)parkBool = true;

            closeClaw();

            telemetry.addData("Starting Position (right: right d-pad, left: left d-pad",right ? "right" : "left");
            telemetry.addData("Wait (no wait: b, wait: x)",wait);
            telemetry.addData("Park? (no park: a, park: y)",parkBool);
            telemetry.update();
        }
        waitForStart();

        if(!isStopRequested()) {

            startPose = right ? rightPos : leftPos;
            drive.setPoseEstimate(startPose);

            TrajectorySequence waitSeq = drive.trajectorySequenceBuilder(startPose)
                    .waitSeconds(7)
                    .build();

            TrajectorySequence leftTraj = drive.trajectorySequenceBuilder(startPose)
                    .lineToConstantHeading(new Vector2d(36, 36)) // Strafe to side
                    .lineToConstantHeading(new Vector2d(36, 13)) // Drive past blocks
//                                .forward(28)
                    .lineToConstantHeading(new Vector2d(44, 13)) // Strafe to first
//                                .strafeRight(12)
                    .lineToConstantHeading(new Vector2d(44, (13+46))) // Push first
//                                .back(42)
                    .lineToConstantHeading(new Vector2d(44, 13)) // Go back
//                                .forward(42)
                    .lineToConstantHeading(new Vector2d(44+10, 13)) // Strafe to Second
//                                .strafeRight(10)
                    .lineToConstantHeading(new Vector2d(44+10, 13+40)) // Push second
//                                .back(42)
                    .lineToConstantHeading(new Vector2d(44+10, 13)) // Go back
//                                .forward(42)
                    .lineToConstantHeading(new Vector2d(44+18, 13)) // Strafe to third
//                                .strafeRight(5)
                    .lineToConstantHeading(new Vector2d(44+18, 13+38)) // Push third
                    .lineToLinearHeading(new Pose2d(34, 12, Math.toRadians(180)))
                    .forward(9)
                    .UNSTABLE_addTemporalMarkerOffset(-1.0, () -> {
                        arm(3600, 5000);
                    })
                    .waitSeconds(3)
                    .build();

            TrajectorySequence rightTraj = drive.trajectorySequenceBuilder(startPose)
                    .lineToConstantHeading(new Vector2d(-36, 36)) // Strafe to side
                    .lineToConstantHeading(new Vector2d(-36, 13)) // Drive past blocks
//                                .forward(28)
                    .lineToConstantHeading(new Vector2d(-44, 13)) // Strafe to first
//                                .strafeRight(12)
                    .lineToConstantHeading(new Vector2d(-44, (13+42))) // Push first
//                                .back(42)
                    .lineToConstantHeading(new Vector2d(-44, 13)) // Go back
//                                .forward(42)
                    .lineToConstantHeading(new Vector2d(-44-10, 13)) // Strafe to Second
//                                .strafeRight(10)
                    .lineToConstantHeading(new Vector2d(-44-10, 13+42)) // Push second
//                                .back(42)
                    .lineToConstantHeading(new Vector2d(-44-10, 13)) // Go back
//                                .forward(42)
                    .lineToConstantHeading(new Vector2d(-44-15, 13)) // Strafe to third
//                                .strafeRight(5)
                    .lineToConstantHeading(new Vector2d(-44-15, 13+42)) // Push third

                    .build();


            if(wait)drive.followTrajectorySequence(waitSeq);

            drive.followTrajectorySequence(right ? rightTraj : leftTraj);

//            Pose2d newPose = new Pose2d(startPose.getX(),10*(blue?1:-1),-startPose.getHeading());
//            if(two&&!right)drive.setPoseEstimate(newPose);

//
//            TrajectorySequence park = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
//                    .lineTo(new Vector2d(38,blue&&left ? 54 : blue ? 16 : left ? -16 : -54))
//                    .splineToConstantHeading(new Vector2d(38+24,blue&&left ? 60 : blue ? 10 : left ? -10 : -60),0)
//                    .build();
//            if(parkBool)drive.followTrajectorySequence(park);

            telemetry.addData("Starting Position",right ? "right" : "left");
            telemetry.addData("Wait",wait);
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

    public void armDown(double velocity) {arm(8700, velocity);
        lift(0,5000);}

    public void armUp(double velocity) {arm(1000, velocity);}

    public void armWall(double velocity) {arm(7000, velocity);
        lift(0, velocity);
    }

    public void basket(double velocity) {arm(4023, velocity);
        lift(6000, velocity);}

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

    public void liftUp(double velocity) {lift(1000, velocity);}

    public void liftDown(double velocity) {lift(0, velocity);}

    public void extender(int pos, double velocity) {
        extender.setTargetPosition(pos);
        extender.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extender.setVelocity(velocity);

        telemetry.addData("Extending to",pos);
        telemetry.update();
    }

    public void openClaw() {
        claw.setPosition(0.4);
    }

    public void closeClaw() {
        claw.setPosition(0.8);
    }



}