package org.firstinspires.ftc.teamcode.xExampleCode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="GOOD DRIVE CODE", group="Iterative Opmode")
public class ExampleCopyMainDriveCode extends OpMode
{
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor rightFrontDrive, rightBackDrive, leftFrontDrive, leftBackDrive, frontLift, mainLift, relicArmOut, relicArmIn;

    private Servo leftArm, rightArm, colorArm, topLeftArm, topRightArm;

    private float drivePowerRY, drivePowerRX, drivePowerLY, drivePowerLX;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        //<editor-fold desc="Designating hardware names">

        //Designating motor names ("exampleName" will be used in the phone code area)
        rightFrontDrive = hardwareMap.dcMotor.get("rightFrontDrive"); //Right drive motors
        rightBackDrive = hardwareMap.dcMotor.get("rightBackDrive");

        leftFrontDrive = hardwareMap.dcMotor.get("leftFrontDrive"); //Left drive motors
        leftBackDrive = hardwareMap.dcMotor.get("leftBackDrive");

        leftFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE); //Setting reverse direction to account for spin
        leftBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLift = hardwareMap.dcMotor.get("frontLift"); //Outer lift that raises the grabber
        mainLift = hardwareMap.dcMotor.get("mainLift"); //Inner lift that raising the front lift motor
        
        leftArm = hardwareMap.servo.get("leftArm"); //Servo for left lift arm
        rightArm = hardwareMap.servo.get("rightArm"); //Servo for right lift arm
        colorArm = hardwareMap.servo.get("colorArm"); //Servo to move the color sensor
        topLeftArm = hardwareMap.servo.get("topLeftArm"); //Servo for the left secondary glyph arm
        topRightArm = hardwareMap.servo.get("topRightArm"); //Servo for the right secondary glyph arm

        //</editor-fold>
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();

        leftArm.setPosition(.5);
        rightArm.setPosition(.5);
        colorArm.setPosition(1);
        topLeftArm.setPosition(.5);
        topRightArm.setPosition(.5);
    }
    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */

    @Override
    public void loop() {
        //<editor-fold desc="Controller 1">
        drivePowerRY = gamepad1.right_stick_y;
        drivePowerRX = gamepad1.right_stick_x;

        drivePowerLY = gamepad1.left_stick_y;
        drivePowerLX = gamepad1.left_stick_x;

        double r = Math.hypot(gamepad1.left_stick_x, -gamepad1.left_stick_y);
        double robotAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
        double rightX = gamepad1.right_stick_x;
        final double v1 = r * Math.cos(robotAngle) + rightX;
        final double v2 = r * Math.sin(robotAngle) - rightX;
        final double v3 = r * Math.sin(robotAngle) + rightX;
        final double v4 = r * Math.cos(robotAngle) - rightX;

        leftFrontDrive.setPower(v1);
        rightFrontDrive.setPower(v2);
        leftBackDrive.setPower(v3);
        rightBackDrive.setPower(v4);

        //</editor-fold>

        //<editor-fold desc="Controller 2">

        if (gamepad2.a && leftArm.getPosition() >=.03 && rightArm.getPosition() <= .97) {
            leftArm.setPosition(leftArm.getPosition() - .03);
            rightArm.setPosition(rightArm.getPosition() + .03);
        }
        if (gamepad2.b && leftArm.getPosition() <=.97 && rightArm.getPosition() >= .03) {
            leftArm.setPosition(leftArm.getPosition() + .03);
            rightArm.setPosition(rightArm.getPosition() - .03);
        }

        telemetry.addData("Left Arm", leftArm.getPosition());
        telemetry.addData("Right Arm", rightArm.getPosition());

        if (gamepad1.y && colorArm.getPosition() >=.01) {
            colorArm.setPosition(colorArm.getPosition() - .01);
        }
        if (gamepad1.x && colorArm.getPosition() <= .99) {
            colorArm.setPosition(colorArm.getPosition() + .01);
        }

        telemetry.addData("Color Arm", colorArm.getPosition());

        if (gamepad2.y && topLeftArm.getPosition() >= .01 && topRightArm.getPosition() <= .99) {
            topLeftArm.setPosition(topLeftArm.getPosition() - .01);
            topRightArm.setPosition(topRightArm.getPosition() + .01);
        }
        if (gamepad2.x && topLeftArm.getPosition() <= .99 && topRightArm.getPosition() >=.01) {
            topLeftArm.setPosition(topLeftArm.getPosition() + .01);
            topRightArm.setPosition(topRightArm.getPosition() - .01);
        }

        telemetry.addData("topLeftArm", topLeftArm.getPosition());
        telemetry.addData("topRightArm", topRightArm.getPosition());

        if (gamepad2.left_trigger > 0) {
            frontLift.setPower(.4);
        }
        if (gamepad2.right_trigger > 0) {
            frontLift.setPower(-.3);
        }
        if (gamepad2.left_trigger == 0 && gamepad2.right_trigger == 0)
            frontLift.setPower(0);

        telemetry.addData("Front Lift Pow", frontLift.getPower());

        if (gamepad2.right_bumper) {
            mainLift.setPower(.4);
        }
        if (gamepad2.left_bumper) {
            mainLift.setPower(-.3);
        }
        if (!gamepad2.left_bumper && !gamepad2.right_bumper)
            mainLift.setPower(0);

        telemetry.addData("Main Lift Pow", mainLift.getPower());

        //</editor-fold>

        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
