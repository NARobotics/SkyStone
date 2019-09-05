package org.firstinspires.ftc.teamcode.aMainCode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="ReverseEncoderAuto", group="Linear Opmode")
//@Disabled
public class LinearAutoMode extends LinearOpMode {

    private DcMotor rightFrontDrive, leftFrontDrive, rightBackDrive, leftBackDrive, outake, intake, lift;

    private Servo arm;

    private ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        rightFrontDrive = hardwareMap.dcMotor.get("rightFrontDrive"); //Right drive motors
        rightBackDrive = hardwareMap.dcMotor.get("rightBackDrive");

        leftFrontDrive = hardwareMap.dcMotor.get("leftFrontDrive"); //Left drive motors
        leftBackDrive = hardwareMap.dcMotor.get("leftBackDrive");

        lift = hardwareMap.dcMotor.get("lift");

        outake = hardwareMap.dcMotor.get("outake");

        intake = hardwareMap.dcMotor.get("intake");

        arm = hardwareMap.servo.get("arm");

        leftFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE); //Setting reverse direction to account for spin
        leftBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);



        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)

            telemetry.addData("Status", "Run Time: " + runtime.toString());

            //brings the robot down from hanging on the lander

            lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            //test number
            lift.setTargetPosition(-12000);

            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            lift.setPower(1);

            while(lift.isBusy())
            {

            }
            lift.setPower(0);

            //brings the box up so it doesn't catch on the ground while the robot moves

            //test idea of not including while statement

            //add sleep command?

            leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            //add two negatives, know that we completely guessed as to which ones they should be on
            //decrease these vales if it ends up hitting the placed out jewels

            leftFrontDrive.setTargetPosition(4000);
            rightFrontDrive.setTargetPosition(-4000);
            leftBackDrive.setTargetPosition(-4000);
            rightBackDrive.setTargetPosition(4000);

            leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            leftFrontDrive.setPower(.5);
            leftBackDrive.setPower(.5);
            rightFrontDrive.setPower(.5);
            rightBackDrive.setPower(.5);



            while(leftFrontDrive.isBusy() && leftBackDrive.isBusy() && rightFrontDrive.isBusy() &&rightBackDrive.isBusy())
            {

            }

            leftFrontDrive.setPower(0);
            leftBackDrive.setPower(0);
            rightFrontDrive.setPower(0);
            rightBackDrive.setPower(0);

            leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            leftFrontDrive.setTargetPosition(-5000);
            rightFrontDrive.setTargetPosition(-5000);
            leftBackDrive.setTargetPosition(-5000);
            rightBackDrive.setTargetPosition(-5000);

            leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            leftFrontDrive.setPower(.5);
            leftBackDrive.setPower(.5);
            rightFrontDrive.setPower(.5);
            rightBackDrive.setPower(.5);
            lift.setPower(.5);



            while(leftFrontDrive.isBusy() && leftBackDrive.isBusy() && rightFrontDrive.isBusy() &&rightBackDrive.isBusy())
            {

            }

            leftFrontDrive.setPower(0);
            leftBackDrive.setPower(0);
            rightFrontDrive.setPower(0);
            rightBackDrive.setPower(0);
            lift.setPower(0);

            leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            leftFrontDrive.setTargetPosition(-1500);
            rightFrontDrive.setTargetPosition(0);
            leftBackDrive.setTargetPosition(-1500);
            rightBackDrive.setTargetPosition(0);

            leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            leftFrontDrive.setPower(.5);
            leftBackDrive.setPower(.5);
            rightFrontDrive.setPower(0);
            rightBackDrive.setPower(0);



            while(leftFrontDrive.isBusy() && leftBackDrive.isBusy())
            {

            }

            leftFrontDrive.setPower(0);
            leftBackDrive.setPower(0);
            rightFrontDrive.setPower(0);
            rightBackDrive.setPower(0);
            lift.setPower(0);

            leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            leftFrontDrive.setTargetPosition(-2000);
            rightFrontDrive.setTargetPosition(-2000);
            leftBackDrive.setTargetPosition(-2000);
            rightBackDrive.setTargetPosition(-2000);

            leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            leftFrontDrive.setPower(.5);
            leftBackDrive.setPower(.5);
            rightFrontDrive.setPower(.5);
            rightBackDrive.setPower(.5);



            while(leftFrontDrive.isBusy() && leftBackDrive.isBusy() && rightFrontDrive.isBusy() &&rightBackDrive.isBusy())
            {

            }

            leftFrontDrive.setPower(0);
            leftBackDrive.setPower(0);
            rightFrontDrive.setPower(0);
            rightBackDrive.setPower(0);






        /*
            leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


            telemetry.addData("Mode", "waiting");
            telemetry.update();




            telemetry.addData("Mode", "running");
            telemetry.update();

            leftFrontDrive.setTargetPosition(-3738);

            leftFrontDrive.setPower(-.5);
            leftBackDrive.setPower(-.5);
            rightFrontDrive.setPower(-.5);
            rightBackDrive.setPower(-.5);

            while(leftFrontDrive.isBusy() && opModeIsActive()) {
            telemetry.addData("encoder-fwd", leftFrontDrive.getCurrentPosition() + "    busy=" + leftFrontDrive.isBusy());
            telemetry.update();
            idle();



            }

            leftFrontDrive.setPower(0);
            leftBackDrive.setPower(0);
            rightFrontDrive.setPower(0);
            rightBackDrive.setPower(0);


            resetStartTime();


            */

            /*
            rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            */
            /*
            leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFrontDrive.setTargetPosition(1120);
        leftFrontDrive.setPower(1);
        while(leftFrontDrive.isBusy() && opModeIsActive()) {
        //Loop body can be empty
        }
        leftFrontDrive.setPower(0);



        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setTargetPosition(1120);
        rightFrontDrive.setPower(1);
        while(rightFrontDrive.isBusy() && opModeIsActive()) {
        //Loop body can be empty
        }
        rightFrontDrive.setPower(0);



        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackDrive.setTargetPosition(1120);
        leftBackDrive.setPower(1);
        while(leftBackDrive.isBusy() && opModeIsActive()) {
        //Loop body can be empty
        }
        leftBackDrive.setPower(0);


        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setTargetPosition(1120);
        rightBackDrive.setPower(1);
        while(rightBackDrive.isBusy() && opModeIsActive()) {
        //Loop body can be empty
        }
        rightBackDrive.setPower(0);





        telemetry.addData("leftFrontDrive", leftFrontDrive.getPower());



        */







            telemetry.update();
        }




}
