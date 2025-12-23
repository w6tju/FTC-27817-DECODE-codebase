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

import static java.lang.String.format;

import android.annotation.SuppressLint;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.resources.accessoryControl;
import org.firstinspires.ftc.teamcode.resources.chassisKinematics.controlRelativity;
import org.firstinspires.ftc.teamcode.resources.driveKinematicController;
import org.firstinspires.ftc.teamcode.resources.robotCfg;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import static org.firstinspires.ftc.teamcode.resources.robotCfg.launcher_intake.assistSpeed;
import static org.firstinspires.ftc.teamcode.resources.robotCfg.launcher_intake.flywheelMaxSpeed;
import static org.firstinspires.ftc.teamcode.resources.robotCfg.launcher_intake.intakeSpeed;

@TeleOp()
public class testOpMode extends LinearOpMode {

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
    public static controlRelativity controlMode = controlRelativity.Field;

    public AnalogInput currentSensor;

    DcMotorEx flywheelLeft;
    DcMotorEx flywheelRight;

    CRServo assistLeft;
    CRServo assistRight;

    CRServo intakeLeft;
    CRServo intakeRight;

    SparkFunOTOS OTOS;

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
        runtime.reset();

        OTOS = hardwareMap.get(SparkFunOTOS.class,"OTOS");
        OTOS.begin();
        OTOS.calibrateImu();


        flywheelLeft = (DcMotorEx)hardwareMap.get(DcMotor.class,"flywheelLeft");
        flywheelRight = (DcMotorEx)hardwareMap.get(DcMotor.class,"flywheelRight");

        intakeLeft = hardwareMap.get(CRServo.class,"intakeLeft");
        intakeRight = hardwareMap.get(CRServo.class,"intakeRight");

        assistLeft = hardwareMap.get(CRServo.class,"assistLeft");
        assistRight = hardwareMap.get(CRServo.class,"assistRight");

        flywheelLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheelRight.setDirection(DcMotorSimple.Direction.FORWARD);

        flywheelLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheelRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        //Main loop (runs as long as you are in op mode)\\
        while (opModeIsActive()) {
            flywheelLeft.setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,new PIDCoefficients(robotCfg.flywheelPID.P, robotCfg.flywheelPID.I, robotCfg.flywheelPID.D));
            flywheelRight.setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,new PIDCoefficients(robotCfg.flywheelPID.P, robotCfg.flywheelPID.I, robotCfg.flywheelPID.D));

            flywheelLeft.setVelocity(flywheelMaxSpeed);
            flywheelRight.setVelocity(flywheelMaxSpeed);

            assistLeft.setPower(assistSpeed);
            assistRight.setPower(assistSpeed);

            intakeLeft.setPower(intakeSpeed);
            intakeRight.setPower(intakeSpeed);



            //region Telemetry
            //Telemetry (shows up on driver hub and FTCDashboard)\\
            panelsTelemetry.addData("Status", "Run Time: " + runtime.toString());
            panelsTelemetry.addData("Current",(format("%.2fA",(currentSensor.getVoltage()/3.3)*50)));
            panelsTelemetry.addData("flywheelLeftTPS",flywheelLeft.getVelocity());
            panelsTelemetry.addData("flywheelRightTPS",flywheelRight.getVelocity());
            panelsTelemetry.addData("flywheelLeftAMPS",flywheelLeft.getCurrent(CurrentUnit.AMPS));
            panelsTelemetry.addData("flywheelRightAMPS",flywheelRight.getCurrent(CurrentUnit.AMPS));
            panelsTelemetry.addData("Position",OTOS.getPosition());
            panelsTelemetry.addData("Rotation",OTOS);
            panelsTelemetry.update(telemetry);
            //endregionr
        }
    }

    float responseCurve(float x) {
        float linear = .25f*x;
        float curve = .75f*(float)Math.pow(x,7);
        return linear+curve;
    }
}