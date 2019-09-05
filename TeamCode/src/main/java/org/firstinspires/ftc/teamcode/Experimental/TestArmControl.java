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

package org.firstinspires.ftc.teamcode.Experimental;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Test Arm Control", group="Iterative Opmode")
@Disabled
public class TestArmControl extends OpMode
{
    private ElapsedTime runtime = new ElapsedTime();

    //private CRServo buttonPusher;

    private Servo clawHand, clawWrist;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        //<editor-fold desc="Designating hardware names">



        //relicArmIn = hardwareMap.dcMotor.get("relicArmIn"); //Motor used to retract the expendable relic arm
        //relicArmOut = hardwareMap.dcMotor.get("relicArmOut"); //Motor used to extend the relic arm


        //Designating servo names
        clawHand = hardwareMap.servo.get("clawHand"); //Servo for left lift arm
        clawWrist = hardwareMap.servo.get("clawWrist"); //Servo for right lift arm


        //</editor-fold>



        //<editor-fold desc="Test">
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

    }
    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */

    @Override
    public void loop() {
        //<editor-fold desc="Controller 1">








        //</editor-fold>

        //<editor-fold desc="Controller 2">
        /*if (gamepad2.a) {
            leftArm.setPosition(1);
            rightArm.setPosition(0);
        }
        if (gamepad2.b) {
            leftArm.setPosition(0);
            rightArm.setPosition(1);
        }*/

        if (gamepad2.a && clawHand.getPosition() >=.03 && clawHand.getPosition() <= .97) {
            clawHand.setPosition(clawHand.getPosition() - .03);
            clawHand.setPosition(clawHand.getPosition() + .03);
        }
        if (gamepad2.b && clawHand.getPosition() <=.97 && clawHand.getPosition() >= .03) {
            clawHand.setPosition(clawHand.getPosition() + .03);
            clawHand.setPosition(clawHand.getPosition() - .03);
        }

        if (gamepad2.y && clawWrist.getPosition() >= .03 && clawWrist.getPosition() <= .97) {
            clawWrist.setPosition(clawWrist.getPosition() - .03);
            clawWrist.setPosition(clawWrist.getPosition() + .03);
        }
        if (gamepad2.x && clawWrist.getPosition() <= .97 && clawWrist.getPosition() >=.03) {
            clawWrist.setPosition(clawWrist.getPosition() + .03);
            clawWrist.setPosition(clawWrist.getPosition() - .03);
        }

        telemetry.addData("clawWrist", clawWrist.getPosition());
        telemetry.addData("clawHand", clawHand.getPosition());



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
