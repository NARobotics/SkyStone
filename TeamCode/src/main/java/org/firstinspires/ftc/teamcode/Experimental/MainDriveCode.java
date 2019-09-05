package org.firstinspires.ftc.teamcode.aMainCode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="GOOD DRIVE CODE", group="Iterative Opmode")
public class MainDriveCode extends OpMode
{
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor rightFrontDrive, rightBackDrive, leftFrontDrive, leftBackDrive;
    //, baseLeftArm, baseRightArm, secondLeftArm, secondRightArm;

    //private CRServo rail;

    private Servo clamp, rail;

    private float drivePowerRY, drivePowerRX, drivePowerLY, drivePowerLX;
    //private double xPower, yPower, liftPower, speedState;
    //private String dropState;

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
        rightBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);

      /*baseLeftArm = hardwareMap.dcMotor.get("baseLeftArm"); //Forward extension motor for the Base
        baseRightArm = hardwareMap.dcMotor.get("baseRightArm"); //Backwards motor for the base

        secondLeftArm = hardwareMap.dcMotor.get("secondLeftArm"); //Forward motor for the second arm
        secondRightArm = hardwareMap.dcMotor.get("secondRightArm"); //Backwards motor for the second arm */


        //Designating servo names
        clamp = hardwareMap.servo.get("clamp"); //Servo for clamp
        rail = hardwareMap.servo.get("rail"); //hook so the robot can lift itself up


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

        clamp.setPosition(.5);
        rail.setPosition(.5);
    }
    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */

    @Override
    public void loop() {
        //<editor-fold desc="Controller 1">
        drivePowerRY = gamepad1.right_stick_y;
        drivePowerRX = gamepad1.right_stick_x;

        drivePowerLY = gamepad1.right_stick_y;
        drivePowerLX = gamepad1.right_stick_x;

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
  if (gamepad2.a && clamp.getPosition() >=.03) {
            clamp.setPosition(clamp.getPosition() - .03);

        }
        if (gamepad2.b && clamp.getPosition() <=.97) {
            clamp.setPosition(clamp.getPosition() + .03);
        }

        if (!gamepad2.x && !gamepad2.y){
            rail.setPosition(.5);
        }
        if (gamepad2.y) {
            rail.setPosition(rail.getPosition() + .03);
        }
        if (gamepad2.x) {
            rail.setPosition(rail.getPosition() - .03);
        }


        telemetry.addData("clamp", clamp.getPosition());
        telemetry.addData("rail", rail.getPosition());

        /* if (gamepad1.left_trigger == 0 && gamepad1.right_trigger ==0)
        {
            baseRightArm.setPower(0);
            baseLeftArm.setPower(0);
        }
        if (gamepad1.left_trigger > 0) {
            baseRightArm.setPower(.4);
            baseLeftArm.setPower(-.4);
        }

        if (gamepad1.right_trigger > 0) {
            baseLeftArm.setPower(.4);
            baseRightArm.setPower(-.4);
        }

        telemetry.addData("Base Left Arm Pow", baseLeftArm.getPower());
        telemetry.addData("Base Right Arm Pow", baseRightArm.getPower());

        if (gamepad1.right_bumper) {
            secondLeftArm.setPower(-.4);
            secondRightArm.setPower(.4);
        }

        if (gamepad1.left_bumper) {
            secondLeftArm.setPower(.4);
            secondRightArm.setPower(-.4);
        }

        if (!gamepad1.right_bumper && !gamepad2.left_bumper)
        {
            secondLeftArm.setPower(0);
            secondRightArm.setPower(0);
        }


        telemetry.addData("Second Left Arm Pow", secondLeftArm.getPower());
        telemetry.addData("Second Right Arm Pow", secondRightArm.getPower());
        */

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
