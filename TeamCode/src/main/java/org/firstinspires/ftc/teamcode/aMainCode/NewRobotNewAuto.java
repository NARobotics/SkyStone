package org.firstinspires.ftc.teamcode.aMainCode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import com.qualcomm.robotcore.hardware.CRServo;

@Autonomous(name="New Robot New Auto", group="Linear Op Mode")
//@Disabled
public class NewRobotNewAuto extends LinearOpMode {

   private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY = "AajGyPH/////AAABmZkuA5xbQkXdp2G9aBJZ2B4W6zMRn8RlywYVE4NcstbzqeqKijsd1uu3G6Ec25sY7QQ+zFNQosb1T0MXUQSr4fRr3rRafM8k5Uj9c2bOECQrLNahDffDQIfiwp3jqHnKsGSdP01VhQ2jMGtrJoZ67tbfkbbBsJbmZ+1JSJvvJ6YG2HJ+Eao5lDRepJ8OmtoeHAVrs6KzXsEHAHWoEMt1nqR0xO4VGy/yaWIPmgrX/W1ZNAecK9CMtQq5bfPCW5/JuxUW4+Yu7IZ/1AeLJ9Xv8qqaiv0NiJRwtASz0njRdvd794Gg075vC04ic5GwmFviqxyEzk86v/wrj09WzPfFzdgZVzlqfTnWAFVwCEn249TR";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the Tensor Flow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    private DcMotor rightFrontDrive, leftFrontDrive, rightBackDrive, leftBackDrive, boxLift, lift, intake;

    private Servo craterArm, marker;

    //private CRServo arm;

    private ElapsedTime runtime = new ElapsedTime();



    private int GoldPosition, distanceShimmy, distanceForward, distanceForward2, distanceShimmy2;





    @Override
    public void runOpMode() {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector())
        {
            initTfod();
        }
        else
        {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();
        waitForStart();

        telemetry.addData("Status", "Initialized");
        telemetry.update();
{
  rightFrontDrive = hardwareMap.dcMotor.get("rightFrontDrive"); //Right drive motors
  rightBackDrive = hardwareMap.dcMotor.get("rightBackDrive");

  leftFrontDrive = hardwareMap.dcMotor.get("leftFrontDrive"); //Left drive motors
  leftBackDrive = hardwareMap.dcMotor.get("leftBackDrive");

  intake = hardwareMap.dcMotor.get("intake"); //intake motor

  lift = hardwareMap.dcMotor.get("lift");

  boxLift = hardwareMap.dcMotor.get("boxLift");

       // arm = hardwareMap.CRServo.get("arm"); //Servo for the outake arm

        craterArm = hardwareMap.servo.get("craterArm"); //servo for the color arm

        marker = hardwareMap.servo.get("marker");

        leftFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE); //Setting reverse direction to account for spin
        leftBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);
}
        // declares hardware and reverses the left drive motors




        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

            telemetry.addData("Status", "Run Time: " + runtime.toString());

{
                        tfod.activate();
                        while (tfod != null) {
                            // getUpdatedRecognitions() will return null if no new information is available since
                            // the last time that call was made.
                            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                            if (updatedRecognitions != null)
                            {
                              telemetry.addData("# Object Detected", updatedRecognitions.size());
                              telemetry.update();
                               if (updatedRecognitions.size() == 2)
                                {
                                    int goldMineralX = -1;
                                    int silverMineral1X = -1;
                                    int silverMineral2X = -1;
                                    for (Recognition recognition : updatedRecognitions)
                                    {
                                        if (recognition.getLabel().equals(LABEL_GOLD_MINERAL))
                                        {
                                            goldMineralX = (int) recognition.getLeft();
                                        }
                                        else if (silverMineral1X == -1)
                                        {
                                            silverMineral1X = (int) recognition.getLeft();
                                        }
                                        else
                                        {
                                            silverMineral2X = (int) recognition.getLeft();
                                        }
                                    }

                                    if (goldMineralX == -1)
                                    {
                                        GoldPosition = 2;
                                        telemetry.addData("Gold Mineral Position", "Right");
                                    }
                                    else if (goldMineralX > silverMineral1X )
                                    {
                                        GoldPosition = 3;
                                        telemetry.addData("Gold Mineral Position", "Center");
                                    }
                                    else
                                    {
                                        GoldPosition = 1;
                                        telemetry.addData("Gold Mineral Position", "Left");

                                    }
                                    tfod = null;
                                }

                              }

                              telemetry.update();
                            }
                        }



            // The actual t-fod code which better work or im gonna have an aneurism
            //Tfod code

{

            lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


            lift.setTargetPosition(-24000);


            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            lift.setPower(4.0);


            while(lift.isBusy())
            {

            }

            lift.setPower(0);

}

            // Lift Code Above DO NOT COMMENT OUT DURING COMPETITION

{
            leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            //add two negatives, know that we completely guessed as to which ones they should be on
            //decrease these vales if it ends up hitting the placed out jewels

            leftFrontDrive.setTargetPosition(-1700);
            rightFrontDrive.setTargetPosition(-1700);
            leftBackDrive.setTargetPosition(-1700);
            rightBackDrive.setTargetPosition(-1700);

            leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            leftFrontDrive.setPower(.7);
            leftBackDrive.setPower(.7);
            rightFrontDrive.setPower(.7);
            rightBackDrive.setPower(.7);



            while(leftFrontDrive.isBusy() && leftBackDrive.isBusy() && rightFrontDrive.isBusy() &&rightBackDrive.isBusy())
            {

            }

            leftFrontDrive.setPower(0);
            leftBackDrive.setPower(0);
            rightFrontDrive.setPower(0);
            rightBackDrive.setPower(0);
}
            // drives back away from the lander

{
            if(GoldPosition == 1) //gold is left
            {
              distanceShimmy = 2500;
              //distanceForward = ;
              //distanceForward2 = ;
              //distanceShimmy2 = ;
            }

            if(GoldPosition == 2) //gold is right
            {
              distanceShimmy = -2500;
              //distanceForward = ;
              //distanceForward2 = ;
              //distanceShimmy2 = ;
            }

            if(GoldPosition == 3) //gold is center
            {
              distanceShimmy = 600;
              //distanceForward = ;
              //distanceForward2 = ;
              //distanceShimmy2 = ;
            }
}

            // sets shimmy and forward drive distances based on supposed position of the gold

{
            leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            leftFrontDrive.setTargetPosition(distanceForward);
            rightFrontDrive.setTargetPosition(-distanceForward);
            leftBackDrive.setTargetPosition(-distanceForward);
            rightBackDrive.setTargetPosition(distanceForward);

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
}
            // shimmys it into prime t-posing position

{
            leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            leftFrontDrive.setTargetPosition(-1500);
            rightFrontDrive.setTargetPosition(-1500);
            leftBackDrive.setTargetPosition(-1500);
            rightBackDrive.setTargetPosition(-1500);

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

}
            // t-poses on the gold mineral to establish dominance (drives backwards and knocks off the gold)







/*
{
  leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
  rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
  leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
  rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

  leftFrontDrive.setTargetPosition();
  rightFrontDrive.setTargetPosition();
  leftBackDrive.setTargetPosition();
  rightBackDrive.setTargetPosition();

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
}

            // sends my child forward after hitting a mineral to a certain distance

{
  leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
  rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
  leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
  rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

  leftFrontDrive.setTargetPosition();
  rightFrontDrive.setTargetPosition(-);
  leftBackDrive.setTargetPosition();
  rightBackDrive.setTargetPosition(-);

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
}
            // turns to be perpendicular to the wall

{
  leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
  rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
  leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
  rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

  leftFrontDrive.setTargetPosition(distanceForward2);
  rightFrontDrive.setTargetPosition(distanceForward2);
  leftBackDrive.setTargetPosition(distanceForward2);
  rightBackDrive.setTargetPosition(distanceForward2);

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
}
            // drives the robot to kiss the wall (muwah)

{
  leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
  rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
  leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
  rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

  leftFrontDrive.setTargetPosition();
  rightFrontDrive.setTargetPosition();
  leftBackDrive.setTargetPosition();
  rightBackDrive.setTargetPosition();

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



  marker.setPosition(.9);
}
            //shimmys to the marker drop position and drops it

{
  leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
  rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
  leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
  rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

  leftFrontDrive.setTargetPosition();
  rightFrontDrive.setTargetPosition();
  leftBackDrive.setTargetPosition();
  rightBackDrive.setTargetPosition();

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
}
            //shimmys to the crater
*/









{
              leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
              rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
              leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
              rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

              leftFrontDrive.setTargetPosition(1400);
              rightFrontDrive.setTargetPosition(1400);
              leftBackDrive.setTargetPosition(1400);
              rightBackDrive.setTargetPosition(1400);

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
}
            // drives out of t-posing


{
    leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            leftFrontDrive.setTargetPosition(-distanceForward);
            rightFrontDrive.setTargetPosition(distanceForward);
            leftBackDrive.setTargetPosition(distanceForward);
            rightBackDrive.setTargetPosition(-distanceForward);

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
}

            //resets to directly infront of lander

{
            leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            leftFrontDrive.setTargetPosition(2000);
            rightFrontDrive.setTargetPosition(-2000);
            leftBackDrive.setTargetPosition(2000);
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
}

            // first turn - 90 degrees

{
    leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            leftFrontDrive.setTargetPosition(3300);
            rightFrontDrive.setTargetPosition(3300);
            leftBackDrive.setTargetPosition(3300);
            rightBackDrive.setTargetPosition(3300);

            leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            leftFrontDrive.setPower(.7);
            leftBackDrive.setPower(.7);
            rightFrontDrive.setPower(.7);
            rightBackDrive.setPower(.7);




            while(leftFrontDrive.isBusy() && leftBackDrive.isBusy() && rightFrontDrive.isBusy() &&rightBackDrive.isBusy())
            {

            }

            leftFrontDrive.setPower(0);
            leftBackDrive.setPower(0);
            rightFrontDrive.setPower(0);
            rightBackDrive.setPower(0);
}
            //drive to drop zone

{
            leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            leftFrontDrive.setTargetPosition(500);
            rightFrontDrive.setTargetPosition(-500);
            leftBackDrive.setTargetPosition(500);
            rightBackDrive.setTargetPosition(-500);

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
}
            //turn a lil bit to drop

{
    marker.setPosition(.4);
    runtime.reset();
    while(runtime.seconds() < .27)
    {

    }
}
            //drops the marker

{
   leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            leftFrontDrive.setTargetPosition(3400);
            rightFrontDrive.setTargetPosition(-3400);
            leftBackDrive.setTargetPosition(3400);
            rightBackDrive.setTargetPosition(-3400);

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
}

            //turns the rest of the way to face away



{
            leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            leftFrontDrive.setTargetPosition(-3200);
            rightFrontDrive.setTargetPosition(-3200);
            leftBackDrive.setTargetPosition(-3200);
            rightBackDrive.setTargetPosition(-3200);

            leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            leftFrontDrive.setPower(.7);
            leftBackDrive.setPower(.7);
            rightFrontDrive.setPower(.7);
            rightBackDrive.setPower(.7);




            while(leftFrontDrive.isBusy() && leftBackDrive.isBusy() && rightFrontDrive.isBusy() &&rightBackDrive.isBusy())
            {

            }

            leftFrontDrive.setPower(0);
            leftBackDrive.setPower(0);
            rightFrontDrive.setPower(0);
            rightBackDrive.setPower(0);
}
            //drives backwards to crater

{

  craterArm.setPosition(1.2);

}
            //P E N E T R A T E S the crater with the crater arm




    }


    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
            "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }

}
