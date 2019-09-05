package org.firstinspires.ftc.teamcode.OldCode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is one example of a way to do autonomous. I do not recommend it. You will not have a good time.
 */

@Autonomous(name="BadShit", group="Linear Opmode")
//@Disabled
public class LinearAutoMode extends LinearOpMode {

    private DcMotor rightFrontDrive, leftFrontDrive, rightBackDrive, leftBackDrive;

    private ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        rightFrontDrive = hardwareMap.dcMotor.get("rightFrontDrive"); //Right drive motors
        rightBackDrive = hardwareMap.dcMotor.get("rightBackDrive");

        leftFrontDrive = hardwareMap.dcMotor.get("leftFrontDrive"); //Left drive motors
        leftBackDrive = hardwareMap.dcMotor.get("leftBackDrive");

        leftFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE); //Setting reverse direction to account for spin
        leftBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
            while(runtime.seconds() < 1.3)
            {
                rightFrontDrive.setPower(.5);
                rightBackDrive.setPower(.5);
                leftFrontDrive.setPower(.5);
                leftFrontDrive.setPower(.5);
                //rightButton.setPower(.5);
                telemetry.update();
            }
            rightFrontDrive.setPower(0);
            rightBackDrive.setPower(0);
            leftFrontDrive.setPower(0);
            leftBackDrive.setPower(0);
            telemetry.update()
            while (runtime.seconds() < 2) {
                rightFrontDrive.setPower(1);
                rightBackDrive.setPower(1);
                leftFrontDrive.setPower(.2);
                leftFrontDrive.setPower(.2);
                telemetry.update();
            }
            rightFrontDrive.setPower(0);
            rightBackDrive.setPower(0);
            leftFrontDrive.setPower(0);
            leftBackDrive.setPower(0);
            telemetry.update();

        }
    }
}
