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

package org.firstinspires.ftc.teamcode.OpModes.TELEOP;

import static org.firstinspires.ftc.teamcode.resources.accessoryControl.*;

import static java.lang.String.*;

import android.annotation.SuppressLint;

import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.resources.accessoryControl;
import org.firstinspires.ftc.teamcode.resources.chassisKinematics.controlRelativity;
import org.firstinspires.ftc.teamcode.resources.driveKinematicController;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.field.PanelsField;
import com.bylazar.field.Style;
import com.bylazar.field.FieldImage;

@TeleOp()
public class MecanumTeleOp extends LinearOpMode {

    boolean loader;
    boolean mode_down = false;
    //Declare OpMode members.
    ElapsedTime runtime = new ElapsedTime();
    boolean last30 = false;
    boolean last10 = false;
    boolean last10Buzz = false;
    double lastBuzzTime;
    private AprilTagProcessor aprilTag;
    driveKinematicController controller;
    public static controlRelativity controlMode = controlRelativity.Robot ;

    public AnalogInput currentSensor;

    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() {
        TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        accessoryControl accessoryController = new accessoryControl(hardwareMap,false);
        currentSensor = hardwareMap.get(AnalogInput.class,"currentSense");
        controller = new driveKinematicController();
        controller.init(hardwareMap);
        panelsTelemetry.addData("Status", "Initialized");
        panelsTelemetry.update(telemetry);

        //waits for start button on driver hub\\
        waitForStart();
        accessoryController.atStart();
        runtime.reset();
        //Main loop (runs as long as you are in op mode)\\
        while (opModeIsActive()) {
            //region Inputs
            double Drive,Drift,Turn;
            Turn = -gamepad1.right_stick_x * 1;// steering input
            if (controlMode == controlRelativity.Field) {
                Drift = (gamepad1.left_stick_y * 1); // forward input
                Drive = (-gamepad1.left_stick_x * 1);// strafe input
                controller.fieldCentericDrive(Drive,Drift,Turn);
            }
            else {
                Drift = (responseCurve(gamepad1.left_stick_y)); // forward input
                Drive = (responseCurve(-gamepad1.left_stick_x)); // strafe input
                controller.drive(Drive,Drift,Turn); //drive output
            }
            //endregion

            if (gamepad1.right_bumper && !mode_down) {
                if (controlMode == controlRelativity.Field) {
                    controlMode = controlRelativity.Robot;
                } else {
                    controlMode = controlRelativity.Field;
                }
            }

            if (!gamepad1.right_bumper && mode_down) {mode_down = false;}

            accessoryController.RunAccessory(gamepad1,gamepad1);

            //aprilTag = new AprilTagProcessor.Builder()
            //        .setDrawAxes(false)
            //        .setDrawCubeProjection(false)
            //        .setDrawTagOutline(true)
            //        .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
            //        .build();
            //VisionPortal.Builder builder = new VisionPortal.Builder();
            //builder.setCamera(hardwareMap.get(WebcamName.class, "MainCam"));
            //builder.enableLiveView(false);

            //region Telemetry
            //Telemetry (shows up on driver hub and FTCDashboard)\\
            panelsTelemetry.addData("Status", "Run Time: " + runtime.toString());
            panelsTelemetry.addData("Current:",(format("%.2fA",(currentSensor.getVoltage()/3.3)*50)));
            panelsTelemetry.addData("flywheel0TPS",accessoryController.flywheel0.getVelocity());
            panelsTelemetry.addData("flywheel1TPS",accessoryController.flywheel1.getVelocity());
            panelsTelemetry.update(telemetry);
            //endregionr55
        }
    }

    float responseCurve(float x) {
        float linear = .25f*x;
        float curve = .75f*(float)Math.pow(x,7);
        return linear+curve;
    }
}