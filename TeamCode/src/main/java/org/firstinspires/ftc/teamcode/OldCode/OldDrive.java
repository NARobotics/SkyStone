package org.firstinspires.ftc.teamcode.OldCode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


/**
 *  This was the only way we used to do drive code. It consisted of take manual inputs directly form the stick positions.
 *  This isn't great because it forces you to use both sticks to move the robot.
 *  Using a single stick is preferable and easier to maneuver.
 */


@TeleOp(name="OldDrive", group="Iterative Opmode")  // @Autonomous(...) is the other common choice
@Disabled
public class OldDrive extends OpMode {
    private DcMotor rightFrontDrive, rightBackDrive, leftFrontDrive, leftBackDrive;



    private float drivePowerRY, drivePowerRX, drivePowerLY, drivePowerLX;
    private double xPower, yPower, liftPower, speedState;
    private String dropState;

    @Override
    public void init() {

        //Designating motor names ("exampleName" will be used in the phone code area)
        rightFrontDrive = hardwareMap.dcMotor.get("rightFrontDrive"); //Right drive motors
        rightBackDrive = hardwareMap.dcMotor.get("rightBackDrive");

        leftFrontDrive = hardwareMap.dcMotor.get("leftFrontDrive"); //Left drive motors
        leftBackDrive = hardwareMap.dcMotor.get("letBackDrive");

        leftFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE); //Setting reverse direction to account for spin
        leftBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    @Override
    public void loop() {

        //<editor-fold desc="Controller 1">
        drivePowerRY = gamepad1.right_stick_y;
        drivePowerRX = gamepad1.right_stick_x;

        drivePowerLY = gamepad1.left_stick_y;
        drivePowerLX = gamepad1.left_stick_x;




        if (drivePowerRX >= 0) {
            rightFrontDrive.setPower(-drivePowerRX);
            rightBackDrive.setPower(drivePowerRX);

            leftFrontDrive.setPower(drivePowerRX);
            leftBackDrive.setPower(-drivePowerRX);
        }
        if (drivePowerRX <= 0) {
            rightFrontDrive.setPower(-drivePowerRX);
            rightBackDrive.setPower(drivePowerRX);

            leftFrontDrive.setPower(drivePowerRX);
            leftBackDrive.setPower(-drivePowerRX);
        }
        if (drivePowerRY >= 0) {
            rightFrontDrive.setPower(drivePowerRX);
            rightBackDrive.setPower(drivePowerRX);

            leftFrontDrive.setPower(drivePowerRX);
            leftBackDrive.setPower(drivePowerRX);
        }
        if (drivePowerRY <= 0) {
            rightFrontDrive.setPower(drivePowerRX);
            rightBackDrive.setPower(drivePowerRX);

            leftFrontDrive.setPower(drivePowerRX);
            leftBackDrive.setPower(drivePowerRX);
        }

        telemetry.addData("Speed State", speedState);

        //</editor-fold>

        telemetry.update();
    }
}
