package org.firstinspires.ftc.teamcode.tele;

import static org.firstinspires.ftc.teamcode.storage.*;


import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.roadrunnertuning.drive.StandardTrackingWheelLocalizer;

@TeleOp(name = "RRTele", group = "!")
public class RRTele extends LinearOpMode {
    boolean autoOp = true;
    int hold = 0;
    DcMotorEx m1, m2, m3,m4, arm,lift, lift2 ,extender;
    Servo claw;

    enum State {
        APPROACHING_WALL,
        LEAVING_WALL,
        APPROACHING_BASKET,
        LEAVING_BASKET,
        APPROACHING_CHAMBER,
        LEAVING_CHAMBER,
        APPROACHING_ASCENT_ZONE,
        AT_ASCENT_ZONE,
        GRABBED_FROM_ASCENT_ZONE,
        LEAVING_ASCENT_ZONE,
        IDLE
    }
    @Override
    public void runOpMode() {
        m1 = (DcMotorEx) hardwareMap.dcMotor.get("leftFront");
        m2 = (DcMotorEx) hardwareMap.dcMotor.get("leftBack");
        m3 = (DcMotorEx) hardwareMap.dcMotor.get("rightFront");
        m4 = (DcMotorEx) hardwareMap.dcMotor.get("rightBack");

        m1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        m2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        m3.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        m4.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

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

        arm.setDirection(DcMotorSimple.Direction.REVERSE);

        claw = hardwareMap.servo.get("claw");

        StandardTrackingWheelLocalizer myLocalizer = new StandardTrackingWheelLocalizer(hardwareMap);
        myLocalizer.setPoseEstimate(currentPose);
        State currentState = State.IDLE;
        waitForStart();
        while (opModeIsActive()) {
            myLocalizer.update();
            if(autoOp) {

                // Going to wall overrides everything
                if(rectangularConstraint(false, -72, 0, 21, 72, 90, 180)) {
                    toPreset(wall);
                    currentState = State.APPROACHING_WALL;
                } else
                //Going to chamber with closed claw overrides everything
                if(rectangularConstraint(true, -20, 20, 38, 45, 180 + 45, 270 + 45)) {
                    toPreset(chamber);
                    currentState = State.APPROACHING_CHAMBER;
                } else
                // Going to basket with closed claw overrides everything
                if(rectangularConstraint(true, 0, 72, 40, 72, 0, 90)) {
                    toPreset(highBasket);
                    currentState = State.APPROACHING_BASKET;
                } else
                //Leaving basket with open claw overrides everything
                if(currentState == State.APPROACHING_BASKET
                        && (rectangularConstraint(false, 0, 45, 40, 60, 0, 360)
                        || rectangularConstraint(false, 0, 72, 40, 72, 180, 360-60))) {
                    toPreset(floor);
                    currentState = State.LEAVING_BASKET;
                } else
                //Approaching ascent zone with open claw
                if(rectangularConstraint(false, 35, 72, -20, 20, 180-45, 180+45)
                        || rectangularConstraint(false, -72, -35, -20, 20, 45, 360-45)) {
                    toPreset(hover);
                    currentState = State.APPROACHING_ASCENT_ZONE;
                } else
                //At ascent zone
                if(rectangularConstraint(false, 22, 28, -20, 20, 180-45, 180+45)
                        || rectangularConstraint(false, -28, -22, -20, 20, 45, 360-45)) {
                    toPreset(floor);
                    currentState = State.AT_ASCENT_ZONE;
                } else
                //Grabbed from ascent zone
                if(rectangularConstraint(true, 22, 28, -20, 20, 180-45, 180+45)
                        || rectangularConstraint(true, -28, -22, -20, 20, 45, 360-45)) {
                    toPreset(hover);
                    currentState = State.GRABBED_FROM_ASCENT_ZONE;
                } else
                //Approaching ascent zone with open claw
                if(rectangularConstraint(true, 35, 72, -20, 20, 180-45, 180+45)
                        || rectangularConstraint(true, -72, -35, -20, 20, 45, 360-45)) {
                    toPreset(hover);
                    currentState = State.LEAVING_ASCENT_ZONE;    
                } else {
                    currentState = State.IDLE;
                }

                if(hold > 0 && !gamepad2.a) {
                    hold = 0;
                    if(gamepad2.right_bumper || gamepad2.left_bumper || gamepad2.dpad_down ||
                            gamepad2.dpad_up || gamepad2.dpad_left || gamepad2.dpad_right ||
                            gamepad2.left_trigger > 0.5 || gamepad2.right_trigger > 0.5 ||
                            gamepad2.a || gamepad2.b || gamepad2.x || gamepad2.y ||
                            Math.abs(gamepad2.left_stick_x) + Math.abs(gamepad2.left_stick_y)
                                    + Math.abs(gamepad2.right_stick_x) + Math.abs(gamepad2.right_stick_y) > 0.5) {
                        autoOp = false;
                    }
                }

            } else {
                mechanumDrive();

                extender.setPower(-gamepad2.right_stick_y);

                if(gamepad1.left_bumper) claw.setPosition(0.4);
                if(gamepad1.right_bumper) claw.setPosition(0.8);

                if(gamepad2.dpad_down) toPreset(floor);
                else if(gamepad2.dpad_up) toPreset(highBasket);
                else if(gamepad2.dpad_right) toPreset(chamber);
                else if(gamepad2.dpad_left) toPreset(wall);
                else {
                    arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    lift2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    arm.setPower(gamepad2.right_trigger - gamepad2.left_trigger);
                    lift.setPower(-gamepad2.left_stick_y);
                    lift2.setPower(-gamepad2.left_stick_y);
                }

                if(gamepad2.a) {
                    autoOp = true;
                    hold = 1;
                }
            }
            telemetry.addData("AutoOp", autoOp);
            if(autoOp) telemetry.addData("Current state", currentState.name());
            else telemetry.addData("Turn on AutoOp", "Press A on Gamepad 2");
            telemetry.update();
        }
    }

    public void mechanumDrive() {
        double px = gamepad1.left_stick_x;
        double py = -gamepad1.left_stick_y;
        double pa = -gamepad1.right_stick_x;

        double p1 = px + py - pa;
        double p2 = -px + py - pa;
        double p3 = -px + py + pa;
        double p4 = px + py + pa;
        double max = Math.max(1.0, Math.abs(p1));
        max = Math.max(max, Math.abs(p2));
        max = Math.max(max, Math.abs(p3));
        max = Math.max(max, Math.abs(p4));
        p1 /= max;
        p2 /= max;
        p3 /= max;
        p4 /= max;
        m1.setPower(p1);
        m2.setPower(p2);
        m3.setPower(p3);
        m4.setPower(p4);
    }

    public void toPreset(int pos) {
        arm.setTargetPosition(pos);
        int liftPos = (pos == highBasket ? highBasketLift : 0);
        lift.setTargetPosition(liftPos);
        lift2.setTargetPosition(liftPos);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setVelocity(armVel);
        lift.setVelocity(liftVel);
        lift2.setVelocity(liftVel);
        telemetry.addData("Going to preset", (pos == highBasket ? "High Basket" : pos == floor ? "Floor" : pos == chamber ? "Chamber" : pos == wall ? "Wall" : "Hover" ));
    }

    public boolean clawClosed() {
        return claw.getPosition() > 0.5;
    }

    public boolean rectangularConstraint(boolean clawClosed, int xMin, int xMax, int yMin, int yMax, int headMin, int headMax) {
        return clawClosed() == clawClosed && currentPose.getX() > xMin && currentPose.getX() < xMax
                && currentPose.getY() > yMin && currentPose.getY() < yMax
                && currentPose.getHeading() > Math.toRadians(headMin)
                && currentPose.getHeading() < Math.toRadians(headMax);
    }

    public boolean circularConstraint(boolean clawClosed, int x, int y, int radius, int headMin, int headMax) {
        return clawClosed() == clawClosed
                && Math.abs(Math.sqrt(currentPose.getX()*currentPose.getX()+currentPose.getY()*currentPose.getY())
                    -Math.sqrt(x*x+y*y)) < radius
                && currentPose.getHeading() > Math.toRadians(headMin)
                && currentPose.getHeading() < Math.toRadians(headMax);
    }
}