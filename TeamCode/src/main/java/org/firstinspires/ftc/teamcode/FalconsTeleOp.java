package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
public class FalconsTeleOp extends LinearOpMode {
    //Initialize motors, servos, sensors, imus, etc.
    DcMotorEx motorLF, motorRF, motorLB, motorRB;
    // TODO: Uncomment the following line if you are using servos
    //Servo Claw;

    public static MecanumDrive.Params DRIVE_PARAMS = new MecanumDrive.Params();


    // The following code will run as soon as "INIT" is pressed on the Driver Station
    public void runOpMode() {

        //Define those motors and stuff
        //The string should be the name on the Driver Hub
        // Set the strings at the top of the MecanumDrive file; they are shared between TeleOp and Autonomous
        motorLF = (DcMotorEx) hardwareMap.dcMotor.get(DRIVE_PARAMS.leftFrontDriveName);
        motorLB = (DcMotorEx) hardwareMap.dcMotor.get(DRIVE_PARAMS.leftBackDriveName);
        motorRF = (DcMotorEx) hardwareMap.dcMotor.get(DRIVE_PARAMS.rightFrontDriveName);
        motorRB = (DcMotorEx) hardwareMap.dcMotor.get(DRIVE_PARAMS.rightBackDriveName);

        // Use the following line as a template for defining new servos
        //Claw = (Servo) hardwareMap.servo.get("claw");

        //Set them to the correct modes
        //This reverses the motor direction
        // This data is also set at the top of MecanumDrive, for the same reasons as above
        motorLF.setDirection(DRIVE_PARAMS.leftFrontDriveDirection);
        motorLB.setDirection(DRIVE_PARAMS.leftBackDriveDirection);
        motorRF.setDirection(DRIVE_PARAMS.rightFrontDriveDirection);
        motorRB.setDirection(DRIVE_PARAMS.rightBackDriveDirection);

        //This resets the encoder values when the code is initialized
        motorLF.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorLB.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorRF.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorRB.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        //This makes the wheels tense up and stay in position when it is not moving, opposite is FLOAT
        motorLF.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorLB.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorRF.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorRB.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        //This lets you look at encoder values while the OpMode is active
        //If you have a STOP_AND_RESET_ENCODER, make sure to put this below it
        motorLF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        // The program will pause here until the Play icon is pressed on the Driver Station
        waitForStart();

        // opModeIsActive() returns "true" as long as the Stop button has not been pressed on the Driver Station
        while(opModeIsActive()) {

            // Mecanum drive code
            double powerX = 0.0;  // Desired power for strafing           (-1 to 1)
            double powerY = 0.0;  // Desired power for forward/backward   (-1 to 1)
            double powerAng = 0.0;  // Desired power for turning          (-1 to 1)

            // Set the desired powers based on joystick inputs (-1 to 1)
            powerX = gamepad1.left_stick_x;
            powerY = -gamepad1.left_stick_y;
            powerAng = -gamepad1.right_stick_x;

            // Perform vector math to determine the desired powers for each wheel
            double powerLF = powerX + powerY - powerAng;
            double powerLB = -powerX + powerY - powerAng;
            double powerRF = -powerX + powerY + powerAng;
            double powerRB = powerX + powerY + powerAng;

            // Determine the greatest wheel power and set it to max
            double max = Math.max(1.0, Math.abs(powerLF));
            max = Math.max(max, Math.abs(powerRF));
            max = Math.max(max, Math.abs(powerLB));
            max = Math.max(max, Math.abs(powerRB));

            // Scale all power variables down to a number between 0 and 1 (so that setPower will accept them)
            powerLF /= max;
            powerLB /= max;
            powerRF /= max;
            powerRB /= max;

            motorLF.setPower(powerLF);
            motorLB.setPower(powerLB);
            motorRF.setPower(powerRF);
            motorRB.setPower(powerRB);



            // If you want to print information to the Driver Station, use telemetry
            // addData() lets you give a string which is automatically followed by a ":" when printed
            //     the variable that you list after the comma will be displayed next to the label
            // update() only needs to be run once and will "push" all of the added data

            //telemetry.addData("Label", "Information");
            //telemetry.update();

        } // opModeActive loop ends
    }
} // end class