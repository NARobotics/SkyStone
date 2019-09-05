package org.firstinspires.ftc.teamcode.OldCode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This code was designed to work with four motors. It works ok, however, the newer 'AutoVuMarkPathing' is a strictly better version in half as many lines.
 * The new 'AutoVuMarkPathing' has the added feature of accessing VuMark pictures as well as has the additional funciton of horizontal driving.
 */

@Autonomous(name="TestEncoderDrive 3", group="Pushbot")
//@Disabled
public class CustomEncoderAutoDrive02 extends LinearOpMode {

    private DcMotor rightFrontDrive, rightBackDrive, leftFrontDrive, leftBackDrive;
    public static final String TAG = "Vuforia VuMark Sample";
    int var;

        /* Declare OpMode members. */

    private ElapsedTime runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.7;
    static final double     TURN_SPEED              = 0.5;
    int x = 0;

    @Override
    public void runOpMode() {

        //<editor-fold desc="Init Variables">
        rightFrontDrive = hardwareMap.dcMotor.get("rightFrontDrive"); //Right drive motors
        rightBackDrive = hardwareMap.dcMotor.get("rightBackDrive");

        leftFrontDrive = hardwareMap.dcMotor.get("leftFrontDrive"); //Left drive motors
        leftBackDrive = hardwareMap.dcMotor.get("leftBackDrive");


        //</editor-fold>


            /*
             * Initialize the drive system variables.
             * The init() method of the hardware class does all the work here
             */
        //init(hardwareMap);


        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        encoderDrive(DRIVE_SPEED,  3*12, 3*12, 200.0);  // S1: Forward 47 Inches with 5 Sec timeout
        encoderDrive(TURN_SPEED,   15, -15, 200.0);  // S2: Turn Right 12 Inches with 4 Sec timeout
        encoderDrive(TURN_SPEED,   3*12, 3*12, 200.0);  // S2: Turn Right 12 Inches with 4 Sec timeout
        //encoderDrive(DRIVE_SPEED, -10, -10, 200.0);  // S3: Reverse 24 Inches with 4 Sec timeout

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newFrontLeftTarget;
        int newFrontRightTarget;
        int newBackLeftTarget;
        int newBackRightTarget;


        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newFrontLeftTarget = leftFrontDrive.getCurrentPosition() - (int)(leftInches * COUNTS_PER_INCH);
            newBackLeftTarget = leftBackDrive.getCurrentPosition() - (int)(leftInches * COUNTS_PER_INCH);

            newFrontRightTarget = rightFrontDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            newBackRightTarget = rightBackDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);

            leftFrontDrive.setTargetPosition(newFrontLeftTarget);
            leftBackDrive.setTargetPosition(newBackLeftTarget);

            rightFrontDrive.setTargetPosition(newFrontRightTarget);
            rightBackDrive.setTargetPosition(newBackRightTarget);

            // Turn On RUN_TO_POSITION
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            leftFrontDrive.setPower(Math.abs(speed));
            leftBackDrive.setPower(Math.abs(speed));
            rightFrontDrive.setPower(-Math.abs(speed));
            rightBackDrive.setPower(-Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (leftFrontDrive.isBusy() && rightFrontDrive.isBusy() && rightBackDrive.isBusy() && leftBackDrive.isBusy())) {

                telemetry.addData("Value", x);

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newFrontLeftTarget, newBackLeftTarget, newFrontLeftTarget, newBackRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        leftFrontDrive.getCurrentPosition(),
                        rightFrontDrive.getCurrentPosition(),
                        leftBackDrive.getCurrentPosition(),
                        rightBackDrive.getCurrentPosition());
                x++;
                telemetry.update();
            }

            // Stop all motion;
            leftFrontDrive.setPower(0);
            rightFrontDrive.setPower(0);
            leftBackDrive.setPower(0);
            rightBackDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            //leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }
}