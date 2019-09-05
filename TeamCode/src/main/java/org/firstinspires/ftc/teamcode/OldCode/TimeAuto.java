package org.firstinspires.ftc.teamcode.OldCode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is one example of a way to do autonomous. I do not recommend it. You will not have a good time.
 */

@Autonomous(name="BadShit", group="Linear Opmode")
@Disabled
public class TimeAuto extends LinearOpMode {

    private DcMotor rightDrive, leftDrive, ballPicker1, ballShooter;
    private CRServo rightButton, leftButton;

    private ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        rightButton = hardwareMap.crservo.get("right_button_servo");

        rightDrive = hardwareMap.dcMotor.get("right_drive");
        leftDrive = hardwareMap.dcMotor.get("left_drive");
        ballPicker1 = hardwareMap.dcMotor.get("ball_picker_1");
        //ballPicker2 = hardwareMap.dcMotor.get("ball_picker_2");

        ballShooter = hardwareMap.dcMotor.get("ball_shooter");
        rightButton.setPower(0);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
            while(runtime.seconds() < 1.3)
            {
                rightDrive.setPower(.5);
                leftDrive.setPower(.6);
                //rightButton.setPower(.5);
                telemetry.update();
            }

            rightDrive.setPower(0);
            leftDrive.setPower(0);

            while (runtime.seconds() < 2.7) {
                ballShooter.setPower(1);
                telemetry.update();
            }

            ballShooter.setPower(0);

            while (runtime.seconds() < 4.7) {
                ballPicker1.setPower(1);
                telemetry.update();
            }

            ballPicker1.setPower(0);

            while (runtime.seconds() < 6.7) {
                ballShooter.setPower(1);
                telemetry.update();
            }

            ballShooter.setPower(0);

            while (runtime.seconds() < 12) {
                rightDrive.setPower(1);
                leftDrive.setPower(.7);
                telemetry.update();
            }
            rightDrive.setPower(0);
            leftDrive.setPower(0);
            telemetry.update();

            while (runtime.seconds() < 13) {
                rightDrive.setPower(.7);
                leftDrive.setPower(-.7);
                telemetry.update();
            }
            rightDrive.setPower(0);
            leftDrive.setPower(0);
            telemetry.update();

            while (runtime.seconds() < 17) {
                rightDrive.setPower(1);
                leftDrive.setPower(1);
                telemetry.update();
            }


            telemetry.update();

        }
    }
}
