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
package org.firstinspires.ftc.teamcode.aMainCode;

import android.app.Activity;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 *
 * It is important to note that Vuvoria needs a produt key to work
 */

private static final String VUFORIA_KEY = "AajGyPH/////AAABmZkuA5xbQkXdp2G9aBJZ2B4W6zMRn8RlywYVE4NcstbzqeqKijsd1uu3G6Ec25sY7QQ+zFNQosb1T0MXUQSr4fRr3rRafM8k5Uj9c2bOECQrLNahDffDQIfiwp3jqHnKsGSdP01VhQ2jMGtrJoZ67tbfkbbBsJbmZ+1JSJvvJ6YG2HJ+Eao5lDRepJ8OmtoeHAVrs6KzXsEHAHWoEMt1nqR0xO4VGy/yaWIPmgrX/W1ZNAecK9CMtQq5bfPCW5/JuxUW4+Yu7IZ/1AeLJ9Xv8qqaiv0NiJRwtASz0njRdvd794Gg075vC04ic5GwmFviqxyEzk86v/wrj09WzPfFzdgZVzlqfTnWAFVwCEn249TR";

@Autonomous(name="Tensor Flow Testing", group ="Concept")

public class TensorFlowDetection extends LinearOpMode {

    initVuforia();

    private void initVuforia() {
      VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Paramters();

      parameters.vuforiaLicenseKey = VUFORIA_KEY;
      parameters.cameraDirection = CameraDirection.BACK;

      vuforia = ClassFactory.getInstance().createVuforia(parameters);

    }

    if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
      initTfod();
    }
    else {
      telemetry.addData("Sorry!", "This device is not compatible with TFOD");
    }

    TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters();
    //can change value later
    tfodParameters.minimumConfidence = 0.75;

    private void initTfod() {
      int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
      TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
      tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
      tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }

    if (tfod != null) {
      tfod.activate();
    }

    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
    if (updatedRecognitions != null) {
      telemetry.addData("# Object Detected", updatedRecognitions.size());
    }

    if (updatedRecognitions.size() == 3) {
      int goldMineralX = -1;
      int silverMineral1X = -1;
      int silverMineral2X = -1;
      for (Recognition recognition : updatedRecognitions) {
        if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
          goldMineralX = (int) recognition.getLeft();
        } else if (silverMineral1X == -1) {
          silverMineral1X = (int) recognition.getLeft();
        } else {
          silverMineral2X = (int) recognition.getLeft();
        }
      }

    if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1) {
        if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
          telemetry.addData("Gold Mineral Position", "Left");
        } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
          telemetry.addData("Gold Mineral Position", "Right");
        } else {
          telemetry.addData("Gold Mineral Position", "Center");
        }
      }



}
