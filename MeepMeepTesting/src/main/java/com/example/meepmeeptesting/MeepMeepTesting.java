package com.example.meepmeeptesting;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedLight;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.awt.Image;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;

public class MeepMeepTesting {
    public static void main(String[] args) {
        // Declare a MeepMeep instance
        // With a field size of 800 pixels
        int flip = 1;
        int flip2 = 1;
        MeepMeep meepMeep = new MeepMeep(600);
        Pose2d rightPos = new Pose2d(-12 * flip2,62 * flip2,Math.toRadians(-90 + 90 - 90 * flip2));
        Pose2d leftPos = new Pose2d(36 * flip,62 * flip,Math.toRadians(-90 + 90 - 90 * flip));



        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Required: Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(30, 30, Math.toRadians(180), Math.toRadians(180), 13.5)
                // Option: Set theme. Default = ColorSchemeRedDark()
                .setDimensions(15,18)
                .setColorScheme(new ColorSchemeRedLight())
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(leftPos)
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
                                .build()
                );

        RoadRunnerBotEntity mySecondBot = new DefaultBotBuilder(meepMeep)
                // Required: Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(30, 30, Math.toRadians(180), Math.toRadians(180), 14.5)
                // Option: Set theme. Default = ColorSchemeRedDark()
                .setDimensions(15,18)
                .setColorScheme(new ColorSchemeRedDark())
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(rightPos)
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
                                .build()
                );

        Image img = null;
        try { img = ImageIO.read(new File("C:\\Users\\jevgo\\OneDrive\\field-2024-juice-dark.png")); }
        catch(IOException e) {}

        meepMeep.setBackground(img)
//        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                // Background opacity from 0-1
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .addEntity(mySecondBot)
                .start();
    }

    public static void arm(int pos, double velocity) {
    }

    public static void armDown(double velocity) {arm(2000, velocity);}

    public static void armUp(double velocity) {arm(1000, velocity);}

    public static void armOut(double velocity) {arm(7000, 1000);}

    public static void reset(double velocity) {arm(0, velocity);}

    public static void wall(double velocity) {arm(0, velocity);}
    public static void basket(double velocity) {arm(0, velocity);}
    public static void chamber(double velocity) {arm(0, velocity);}
    public static void floor(double velocity) {arm(0, velocity);}

    public static void lift(int pos, double velocity) {
    }

    public static void liftUp(double velocity) {lift(1000, velocity);}

    public static void liftDown(double velocity) {lift(0, velocity);}

    public static void extender(int pos, double velocity) {
    }

    public static void openClaw() {
    }

    public static void closeClaw() {
    }
}