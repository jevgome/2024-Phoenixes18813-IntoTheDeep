package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.roadrunnertuning.drive.SampleMecanumDrive;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.roadrunnertuning.trajectorysequence.TrajectorySequence;


import com.qualcomm.robotcore.hardware.*;

@Autonomous(group = "!auto")
public class MasterAuto extends LinearOpMode {
    DcMotorEx arm,lift, lift2 ,extender;

    DistanceSensor left, front;
    Servo claw;

    int armVel = 50000;
    int liftVel = 50000;

    double DISTANCE_TO_WALL = 20;//in-
    double DISTANCE_TO_SAMPLE = 9.3;//in
    boolean right= true;

    @Override
    public void runOpMode() throws InterruptedException {
        arm = (DcMotorEx) hardwareMap.dcMotor.get("extender");
        arm.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        lift = (DcMotorEx) hardwareMap.dcMotor.get("lift");
        lift.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        lift2 = (DcMotorEx) hardwareMap.dcMotor.get("lift2");
        lift2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        lift2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        extender = (DcMotorEx) hardwareMap.dcMotor.get("arm");
        extender.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        extender.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        lift.setDirection(DcMotorSimple.Direction.REVERSE);
//        lift2.setDirection(DcMotorSimple.Direction.REVERSE);
        arm.setDirection(DcMotorSimple.Direction.REVERSE);

        claw = hardwareMap.servo.get("claw");

        front = hardwareMap.get(DistanceSensor.class, "front_distance");
        left = hardwareMap.get(DistanceSensor.class, "left_distance");

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d rightPos = new Pose2d(-12,62,Math.toRadians(0));
        Pose2d leftPos = new Pose2d(36,64,Math.toRadians(0));
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
                    .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {
                        basket(10000);
                    }) // Arm up, lift up
                    .waitSeconds(1)
                    .lineToLinearHeading(new Pose2d(53, 50, Math.toRadians(60)))
                    .forward(6)
                    .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {
                        openClaw();
                    }) // Claw open
                    .waitSeconds(0.3)

                    // First cycle
                    .back(8)
                    .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {
                        floor(10000);
                    }) // Arm down, lift down
                    .lineToLinearHeading(new Pose2d(50, 51, Math.toRadians(-90)))
                    .forward(6)
                    .UNSTABLE_addTemporalMarkerOffset(0.0, ()->{
                        closeClaw();
                    }) // Claw close
                    .waitSeconds(0.3)
                    .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {
                        basket(10000);
                    }) // Arm up, lift up
                    .waitSeconds(0.8)
                    .lineToLinearHeading(new Pose2d(50, 50, Math.toRadians(55)))
                    .forward(8)
                    .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {
                        openClaw();
                    }) // Claw open
                    .waitSeconds(0.3)
                    .back(8)

                    // Second cycle
                    .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {
                        floor(10000);
                    }) // Arm down, lift down
                    .lineToLinearHeading(new Pose2d(62, 52, Math.toRadians(-90)))
                    .forward(5)
                    .UNSTABLE_addTemporalMarkerOffset(0.0, ()->{
                        closeClaw();
                    }) // Claw close
                    .waitSeconds(0.3)
                    .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {
                        basket(10000);
                    }) // Arm up, lift up
                    .waitSeconds(0.5)
                    .lineToLinearHeading(new Pose2d(50, 50, Math.toRadians(55)))
                    .forward(12)
                    .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {
                        openClaw();
                    }) // Claw open
                    .waitSeconds(0.3)

                    // Last one
                    .UNSTABLE_addTemporalMarkerOffset(0.4, () -> {arm(2500, 10000); lift(0);})
                    .lineToLinearHeading(new Pose2d(49.3, 30.5, Math.toRadians(0)))
                    .build();

            TrajectorySequence rightTraj = drive.trajectorySequenceBuilder(startPose)
                    // Pre-load
                    .addTemporalMarker(0.0, () -> {
                        chamber(armVel);
                    })
                    .lineToLinearHeading(new Pose2d(-6, 38, Math.toRadians(-90)))
                    .forward(3)
//                    .splineToConstantHeading(new Vector2d(-6, 35), Math.toRadians(257.62))
                    .UNSTABLE_addTemporalMarkerOffset(0.0, ()->{openClaw();}) // Open claw
                    .waitSeconds(0.3)

                    // To first
                    .UNSTABLE_addTemporalMarkerOffset(1.1, () -> {
                        wall(10000);
                    }) // lift down, arm reset
                    .back(4)
                    .splineTo(new Vector2d(-34.46, 29.30), Math.toRadians(268.49))
                    .splineToSplineHeading(new Pose2d(-46, 14, Math.toRadians(90.00)), Math.toRadians(180.00))
                    .lineToConstantHeading(new Vector2d(-46, 51)) // Push first

                    // Grab from wall
                    .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {closeClaw();})
                    .waitSeconds(0.3)
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> {chamber(10000);})

                    .lineToLinearHeading(new Pose2d(-8, 40.65, Math.toRadians(-90.00 + 1e-6)))
                    .forward(2)
                    .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {openClaw();})
                    .waitSeconds(0.3)

                    // To second
                    .UNSTABLE_addTemporalMarkerOffset(2.0, () -> {
                        wall(10000);
                    }) // lift down, arm reset
                    .back(4)
                    .lineToLinearHeading(new Pose2d(-46, 47, Math.toRadians(90.00)))

                    // Grab from wall
                    .forward(6.5)
                    .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {closeClaw();})
                    .waitSeconds(0.3)
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> {chamber(10000);})
                    .lineToLinearHeading(new Pose2d(-8, 40.65, Math.toRadians(-90.00+1e-6)))
                    .forward(2)
                    .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {openClaw();})
                    .waitSeconds(0.3)
                    .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {arm(1450);})
                    .back(4)
                    .splineTo(new Vector2d(-34.46, 29.30), Math.toRadians(268.49))
                    .splineToSplineHeading(new Pose2d(-44, 14, Math.toRadians(90.00)), Math.toRadians(180.00))
                    .strafeLeft(8.5)
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> {reset(10000);})
                    .forward(50)


                    .build();

            TrajectorySequence rightTraj2 = drive.trajectorySequenceBuilder(startPose)
//                    // Pre-load
//                    .addTemporalMarker(-0.6, () -> {
//                        chamber(10000);
//                    }) // Arm up, lift up
//                    .splineToConstantHeading(new Vector2d(-8, 35), Math.toRadians(257.62))
//                    .UNSTABLE_addTemporalMarkerOffset(0.0, ()->{openClaw();}) // Open claw
//                    .waitSeconds(0.3)
//
//                    // To first
//                    .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {
//                        reset(10000);
//                    }) // lift down, arm reset
//                    .back(4)
//                    .splineTo(new Vector2d(-34.46, 29.30), Math.toRadians(268.49))
//                    .splineToSplineHeading(new Pose2d(-48.76, 9.04, Math.toRadians(90.00)), Math.toRadians(180.00))
//                    .strafeLeft(10)
//                    .lineToConstantHeading(new Vector2d(-48-10, 51)) // Push second
//                    .lineToConstantHeading(new Vector2d(-48-10, 9.04)) // Go back
//                    .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {
//                        wall(10000);
//                    }) // lift down, arm reset
//                    .strafeRight(10)
//                    .lineToConstantHeading(new Vector2d(-48, 51)) // Push first
//                    // Grab from wall
//                    .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {closeClaw();})
//                    .waitSeconds(0.3)
//                    .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {lift(500, 10000);})
//                    .waitSeconds(0.3)
//                    .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {chamber(10000);})
//                    .lineToLinearHeading(new Pose2d(-10, 40.65, Math.toRadians(-90.00)))
//                    .forward(3)
//                    .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {openClaw();})
//                    .waitSeconds(0.3)
//                    // To second
//                    .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {
//                        wall(10000);
//                    }) // lift down, arm reset
//                    .back(4)
//                    .lineToLinearHeading(new Pose2d(-48, 46, Math.toRadians(90.00)))
//                    .forward(5)
//
//                    // Grab from wall
//                    .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {closeClaw();})
//                    .waitSeconds(0.3)
//                    .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {lift(500, 10000);})
//                    .waitSeconds(0.3)
//                    .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {chamber(10000);})
//                    .lineToLinearHeading(new Pose2d(-12, 40.65, Math.toRadians(-90.00)))
//                    .forward(3)
//                    .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {openClaw();})
//                    .waitSeconds(0.3)
//                    .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {reset(10000);})
//                    .back(10)
                    .lineTo(new Vector2d(-43, 50))
                    .build();

            if(right) {
                drive.followTrajectorySequence(rightTraj);

//                double dFront = front.getDistance(DistanceUnit.INCH);
//                double dLeft = left.getDistance(DistanceUnit.INCH);
//                TrajectorySequence back = drive.trajectorySequenceBuilder(rightTraj.end())
//                        .forward(dFront-DISTANCE_TO_WALL)
//                        .build();

//                drive.followTrajectorySequence(back);

//                drive.setPoseEstimate(new Pose2d(-70.5 + 15.0/2 + dLeft, rightTraj.end().getY(), rightTraj.end().getHeading()));

//                TrajectorySequence obDesk = drive.trajectorySequenceBuilder(rightTraj.end())
//                        // Grab from wall
//                        .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {closeClaw();})
//                        .waitSeconds(0.3)
//                        .UNSTABLE_addTemporalMarkerOffset(0, () -> {chamber(10000);})
//
//                        .lineToLinearHeading(new Pose2d(-8, 40.65, Math.toRadians(-90.00 + 1e-6)))
//                        .forward(2)
//                        .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {openClaw();})
//                        .waitSeconds(0.3)
//
//                        // To second
//                        .UNSTABLE_addTemporalMarkerOffset(2.0, () -> {
//                            wall(10000);
//                        }) // lift down, arm reset
//                        .back(4)
//                        .lineToLinearHeading(new Pose2d(-46, 47, Math.toRadians(90.00)))
//                        .build();

//                dFront = front.getDistance(DistanceUnit.INCH);
//                TrajectorySequence back2 = drive.trajectorySequenceBuilder(obDesk.end())
//                        .forward(dFront-DISTANCE_TO_WALL)
//                        .build();
//
//                drive.followTrajectorySequence(back2);
//
//                drive.setPoseEstimate(obDesk.end());

//                TrajectorySequence obDesk2 = drive.trajectorySequenceBuilder(obDesk.end())
//                        // Grab from wall
//                        .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {closeClaw();})
//                        .waitSeconds(0.3)
//                        .UNSTABLE_addTemporalMarkerOffset(0, () -> {chamber(10000);})
//                        .lineToLinearHeading(new Pose2d(-8, 40.65, Math.toRadians(-90.00+1e-6)))
//                        .forward(2)
//                        .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {openClaw();})
//                        .waitSeconds(0.3)
//                        .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {arm(1450);})
//                        .back(4)
//                        .splineTo(new Vector2d(-34.46, 29.30), Math.toRadians(268.49))
//                        .splineToSplineHeading(new Pose2d(-46, 14, Math.toRadians(90.00)), Math.toRadians(180.00))
//                        .strafeLeft(8.5)
//                        .UNSTABLE_addTemporalMarkerOffset(0, () -> {reset(10000);})
//                        .forward(50)
//                        .build();

//                drive.followTrajectorySequence(obDesk2);
            } else {
                drive.followTrajectorySequence(leftTraj);

//                double dFront= front.getDistance(DistanceUnit.INCH)-DISTANCE_TO_SAMPLE;
//                TrajectorySequence forward = drive.trajectorySequenceBuilder(leftTraj.end())
//                        .forward(dFront < 10 ? dFront : 6.5)
//                        .build();
//
//                drive.followTrajectorySequence(forward);
//
//                drive.setPoseEstimate(leftTraj.end());
//
//                dFront= front.getDistance(DistanceUnit.INCH)-DISTANCE_TO_SAMPLE;
//                if(dFront != 0) {
//                    TrajectorySequence forward2 = drive.trajectorySequenceBuilder(leftTraj.end())
//                            .forward(dFront < 10 ? dFront : 1)
//                            .build();
//
//                    drive.followTrajectorySequence(forward2);
//                    drive.setPoseEstimate(new Pose2d(leftTraj.end().getX() + 6.5, leftTraj.end().getY(), leftTraj.end().getHeading()));
//                }

                TrajectorySequence b = drive.trajectorySequenceBuilder(leftTraj.end())
                        .forward(7.5)
                        .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {closeClaw();})
                        .waitSeconds(0.3)
                        .back(8)
                        .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {
                            basket(10000);
                        }) // Arm up, lift up
                        .waitSeconds(0.5)
                        .lineToLinearHeading(new Pose2d(50, 50, Math.toRadians(55)))
                        .forward(12)
                        .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {
                            openClaw();
                        }) // Claw open
                        .waitSeconds(0.3)
                        .back(6)

                        // Park
                        .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                            lift(0, 10000);
                        })
                        .lineToLinearHeading(new Pose2d(34, 12, Math.toRadians(180)))
                        .forward(10)
                        .UNSTABLE_addTemporalMarkerOffset(-1.0, () -> {
                            arm(1450, 5000);
                        })
                        .waitSeconds(3)
                        .build();

                drive.followTrajectorySequence(b);
            }

//            Pose2d newPose = new Pose2d(startPose.getX(),10*(blue?1:-1),-startPose.getHeading());
//            if(two&&!right)drive.setPoseEstimate(newPose);

//
//            TrajectorySequence park = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
//                    .lineTo(new Vector2d(38,blue&&left ? 54 : blue ? 16 : left ? -16 : -54))
//                    .splineToConstantHeading(new Vector2d(38+24,blue&&left ? 60 : blue ? 10 : left ? -10 : -60),0)
//                    .build();
//            if(parkBool)drive.followTrajectorySequence(park);

            telemetry.addData("Starting Position",right ? "right" : "left");
            telemetry.addData("","");
            telemetry.addData("Arm", arm.getCurrentPosition());
            telemetry.addData("Lift", lift.getCurrentPosition());

            telemetry.update();
        }
    }

    public void arm(int pos) {
        arm.setTargetPosition(pos);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setVelocity(armVel);

        telemetry.addData("Moving Arm to",pos);
        telemetry.update();
    }
    public void arm(int pos, double velocity) {
        arm.setTargetPosition(pos);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setVelocity(velocity);

        telemetry.addData("Moving Arm to",pos);
        telemetry.update();
    }

    public void wall(double velocity) {arm(2070, velocity);
        lift(0);
    }

    public void basket(double velocity) {arm(1350, velocity);
        lift(3200);}

    public void floor(double velocity) {arm(2480, velocity); lift(0);}
    public void chamber(int velocity) {arm(1350, velocity);
        lift(0);}
    public void reset(double velocity) {arm(0, velocity);
        lift(0);}

    public void lift(int pos) {
        lift.setTargetPosition(pos);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setVelocity(liftVel);

        lift2.setTargetPosition(pos);
        lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift2.setVelocity(liftVel);

        telemetry.addData("Lifting to",pos);
        telemetry.update();
    }

    public void lift(int pos, double velocity) {
        lift.setTargetPosition(pos);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setVelocity(velocity);

        lift2.setTargetPosition(pos);
        lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift2.setVelocity(velocity);

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
