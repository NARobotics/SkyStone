/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package org.firstinspires.ftc.teamcode.xExampleCode;

import android.app.Activity;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.ConceptVuforiaNavigation;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;

@Autonomous(name="NEW CLICK THIS ONE Auto", group ="Auto")
@Disabled
public class ExampleCopyAutoRedCrypto extends LinearOpMode {

    private DcMotor rightFrontDrive, rightBackDrive, leftFrontDrive, leftBackDrive;
    private static final String TAG = "Vuforia VuMark Sample";
    boolean found = false;
    boolean done = false;
    int var;
    ColorSensor colorSensor;

    private DcMotor frontLift;

    private Servo leftArm, rightArm, colorArm, topLeftArm, topRightArm;
    private ElapsedTime runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.2;
    static final double     TURN_SPEED              = 0.1;
    int x = 0;

    OpenGLMatrix lastLocation = null;

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    VuforiaLocalizer vuforia;

    @Override public void runOpMode() {

        //<editor-fold desc="Init Variables">
        rightFrontDrive = hardwareMap.dcMotor.get("rightFrontDrive"); //Right drive motors
        rightBackDrive = hardwareMap.dcMotor.get("rightBackDrive");

        leftFrontDrive = hardwareMap.dcMotor.get("leftFrontDrive"); //Left drive motors
        leftBackDrive = hardwareMap.dcMotor.get("leftBackDrive");

        //leftFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE); //Setting reverse direction to account for spin
        //leftBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        colorArm = hardwareMap.servo.get("colorArm"); //Servo to move the color sensor
        leftArm = hardwareMap.servo.get("leftArm"); //Servo for left lift arm
        rightArm = hardwareMap.servo.get("rightArm"); //Servo for right lift arm
        topLeftArm = hardwareMap.servo.get("topLeftArm"); //Servo for the left secondary glyph arm
        topRightArm = hardwareMap.servo.get("topRightArm"); //Servo for the right secondary glyph arm


        frontLift = hardwareMap.dcMotor.get("frontLift");

        //</editor-fold>

        //<editor-fold desc="VuForia Setup">
        /*
         * To start up Vuforia, tell it the view that we wish to use for camera monitor (on the RC phone);
         * If no camera monitor is desired, use the parameterless constructor instead (commented out below).
         */
        //int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        //VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View, to save power
        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        //License Key
        //parameters.vuforiaLicenseKey = "ATRVz5P/////AAAAGUGDn7ezGUuxhsQ8C1BvtnRCLrOesfpzkg7HM/G2iHEPTMxEijwh60S1vVe9ZwJ/gLyhno4I3iA/nk07byHF2cWnSJlwZUuH3OGyqf5UI1IcB9A532CZpNYt+IX7/CX655084cttkigrSTogcFUJYj3f2fDTTmhiy3vX/Zx8EAVYqBVdVZWJHdr8XfCn56BrYTT5knF/kakEs/bTDaqPwiT1O3GYtWdOS9S90eaUGpyNxOSh4RXLliGE32DstvdlVWiCtHV2hsaQW60ymgJJboNHYaTAzkdTh1BrIQkHU+mB2nQTcsT+Ehb7BjF2EXwGKWWf7n1yArlhe9fli20nd9MqBRc6d6cPT9ubl2dgk16G";

        //Designates back camera
        //parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        //this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        /**
         * Load the data set containing the VuMarks for Relic Recovery. There's only one trackable
         * in this data set: all three of the VuMarks in the game were created from this one template,
         * but differ in their instance id information.
         * @see VuMarkInstanceId
         */
        //VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        //VuforiaTrackable relicTemplate = relicTrackables.get(0);
        //relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        telemetry.addData(">", "Press Play to start");
        telemetry.update();

        //</editor-fold>

        //<editor-fold desc="Color Setup">

        float hsvValues[] = {0F,0F,0F};
        final float values[] = hsvValues;
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(com.qualcomm.ftcrobotcontroller.R.id.RelativeLayout);

        colorSensor = hardwareMap.colorSensor.get("colorSensor");
        colorSensor.enableLed(true);

        //</editor-fold>


        waitForStart();

        colorArm.setPosition(1);
        leftArm.setPosition(1);
        rightArm.setPosition(0);
        topLeftArm.setPosition(1);
        topRightArm.setPosition(0);

        double x = getRuntime();
        double y = getRuntime()+1;

        while (x < y) {
            frontLift.setPower(-.3);
            x = getRuntime();
        }


        //relicTrackables.activate();

        //moveToJewel();
        //colorSense();

        encoderDrive(.3, 60, 60, 15);

        stop();

        /*while (opModeIsActive()) {

            *//**
             * See if any of the instances of {@link relicTemplate} are currently visible.
             * {@link RelicRecoveryVuMark} is an enum which can have the following values:
             * UNKNOWN, LEFT, CENTER, and RIGHT. When a VuMark is visible, something other than
             * UNKNOWN will be returned by {@link RelicRecoveryVuMark#from(VuforiaTrackable)}.
             *//*
            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            if (!found && vuMark != RelicRecoveryVuMark.UNKNOWN) {
                *//* Found an instance of the template. In the actual game, you will probably
                 * loop until this condition occurs, then move on to act accordingly depending
                 * on which VuMark was visible. *//*
                telemetry.addData("VuMark", "%s visible", vuMark);

                *//* For fun, we also exhibit the navigational pose. In the Relic Recovery game,
                 * it is perhaps unlikely that you will actually need to act on this pose information, but
                 * we illustrate it nevertheless, for completeness. *//*
                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener)relicTemplate.getListener()).getPose();
                telemetry.addData("Pose", format(pose));

                *//* We further illustrate how to decompose the pose into useful rotational and
                 * translational components *//*
                if (pose != null) {
                    VectorF trans = pose.getTranslation();
                    Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

                    // Extract the X, Y, and Z components of the offset of the target relative to the robot
                    double tX = trans.get(0);
                    double tY = trans.get(1);
                    double tZ = trans.get(2);

                    // Extract the rotational components of the target relative to the robot
                    double rX = rot.firstAngle;
                    double rY = rot.secondAngle;
                    double rZ = rot.thirdAngle;
                }
                if (vuMark == RelicRecoveryVuMark.LEFT){
                    var = 1;
                }
                if (vuMark == RelicRecoveryVuMark.CENTER){
                    var = 2;
                }
                if (vuMark == RelicRecoveryVuMark.RIGHT){
                    var = 3;
                }
                found = true;
            }
            else if (!found){
                telemetry.addData("VuMark", "not visible");
            }
            else if (found && !done){
                switch (var){
                    case 1: left(); break; //Left
                    case 2: center(); break; //Center
                    case 3: right(); break; //Right

                }
            }
            telemetry.update();
        }*/
    }

    private void left(){
        encoderDriveShimmy(.7, 6, 12, 5, 1 );
        done = true;
    }
    private void center(){
        encoderDrive(.7, 6, 6, 5);
        done = true;
    }
    private void right(){
        encoderDrive(.7, 12, 6, 5);
        done = true;
    }

    private void moveToJewel(){
        encoderDrive(.7, 10, -10, 29);
        //encoderDriveShimmy(.15, 12, 12, 4,0);
    }
    private void colorSense(){
        colorArm.setPosition(.4);
        boolean thing = true;
        while (thing && opModeIsActive()) {
            telemetry.addData("No Color Seen", "NO COLOR");
            telemetry.addData("Red", colorSensor.red());
            telemetry.addData("Blue", colorSensor.blue());
            telemetry.update();

            if (colorSensor.blue() > colorSensor.red()) {
                encoderDrive(.2, -5, -5, 29);
                colorArm.setPosition(1);
                encoderDrive(.2, 5, 5, 29);
                thing = false;
            } else if (colorSensor.red() > colorSensor.blue()) {
                encoderDrive(.2, 5, 5, 29);
                colorArm.setPosition(1);
                encoderDrive(.2, -5, -5, 29);
                thing = false;
            }
            else thing = true;

        }
        encoderDrive(.2, -8, -8, 4);



    }

    /*private void jewelScore(int color){
        switch (color){
            case 0: left(); break; //Not Found
            case 1: left(); break; //Sees Red
            case 2: center(); break; //Sees Blue

        }
    }*/

    private void encoderDrive(double speed, double leftInches, double rightInches, double timeoutS) {
        int newFrontLeftTarget;
        int newFrontRightTarget;
        int newBackLeftTarget;
        int newBackRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newFrontLeftTarget = leftFrontDrive.getCurrentPosition() - (int)(leftInches * COUNTS_PER_INCH);
            newBackLeftTarget = leftBackDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);

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
            leftBackDrive.setPower(-Math.abs(speed));
            rightFrontDrive.setPower(-Math.abs(speed));
            rightBackDrive.setPower(-Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (leftFrontDrive.isBusy() && leftBackDrive.isBusy() && rightFrontDrive.isBusy() && rightBackDrive.isBusy())) {

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

    private void encoderDriveShimmy(double speed, double leftInches, double rightInches, double timeoutS, int direction) {
        int newFrontLeftTarget;
        int newFrontRightTarget;
        int newBackLeftTarget;
        int newBackRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            if (direction == 0) {
                //Shimmy left
                // Determine new target position, and pass to motor controller
                newFrontLeftTarget = leftFrontDrive.getCurrentPosition() - (int) (leftInches * COUNTS_PER_INCH);
                newBackLeftTarget = leftBackDrive.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);

                newFrontRightTarget = rightFrontDrive.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
                newBackRightTarget = rightBackDrive.getCurrentPosition() - (int) (rightInches * COUNTS_PER_INCH);
            }
            else if (direction == 1) {
                //Shimmy right
                // Determine new target position, and pass to motor controller
                newFrontLeftTarget = leftFrontDrive.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
                newBackLeftTarget = leftBackDrive.getCurrentPosition() - (int) (leftInches * COUNTS_PER_INCH);

                newFrontRightTarget = rightFrontDrive.getCurrentPosition() - (int) (rightInches * COUNTS_PER_INCH);
                newBackRightTarget = rightBackDrive.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            }
            else {
                newFrontLeftTarget = leftFrontDrive.getCurrentPosition();
                newBackLeftTarget = leftBackDrive.getCurrentPosition();

                newFrontRightTarget = rightFrontDrive.getCurrentPosition();
                newBackRightTarget = rightBackDrive.getCurrentPosition();
            }

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
                    (leftFrontDrive.isBusy() && rightFrontDrive.isBusy() && rightBackDrive.isBusy() && rightBackDrive.isBusy())) {

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

    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }
}
