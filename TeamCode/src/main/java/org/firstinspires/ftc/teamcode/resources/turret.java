package org.firstinspires.ftc.teamcode.resources;

import static org.firstinspires.ftc.teamcode.resources.robotCfg.turret.*;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

public class turret {
    DcMotorEx azimuth;
    Servo elevation;

    DcMotorEx flywheel0;
    DcMotorEx flywheel1;

    DigitalChannel launchSensor;

    TelemetryManager panelsTelemetry;


    // values
    int flywheelSpeed = 0;
    public boolean ballIn = false;

    private int azimuthPosition = 0;

    public turret(HardwareMap hwMp, TelemetryManager ptel) {
        //region Hardware
        azimuth = hwMp.get(DcMotorEx.class,"azimuth");
        elevation = hwMp.get(Servo.class,"elev");
        flywheel0 = hwMp.get(DcMotorEx.class,"flywheel0");
        flywheel1 = hwMp.get(DcMotorEx.class,"flywheel1");
        launchSensor = hwMp.get(DigitalChannel.class,"launchSensor");
        //endregion

        //region Directions
        flywheel0.setDirection(DcMotorSimple.Direction.FORWARD);
        flywheel1.setDirection(DcMotorSimple.Direction.REVERSE);
        //endregion

        //region Zero power
        flywheel0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheel1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        azimuth.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //endregion

        //region PID
        flywheel0.setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,new PIDCoefficients(robotCfg.flywheelPID.P, robotCfg.flywheelPID.I, robotCfg.flywheelPID.D));
        flywheel1.setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,new PIDCoefficients(robotCfg.flywheelPID.P, robotCfg.flywheelPID.I, robotCfg.flywheelPID.D));
        //endregion

        azimuth.setTargetPositionTolerance(2);
        azimuth.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        launchSensor.setMode(DigitalChannel.Mode.INPUT);

        panelsTelemetry = ptel;
    }

    //automatic
    public void aimToPos(Pose botPos, Pose3D aimTo) {
        
    }

    //manual
    public void adjustTurret(double elevation, double azimuth,int flywheel) {
        int tprAzimuth = motorTpr*aziRatio;
        azimuthPosition = (int)(azimuth)*(tprAzimuth/360);

        flywheelSpeed = flywheel;
    }

    public void passiveRun() {
        azimuth.setTargetPosition(azimuthPosition);
        azimuth.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        azimuth.setVelocity(2700);
        panelsTelemetry.addData("turretPosition",azimuth.getCurrentPosition()/(motorTpr*aziRatio));

        ballIn = launchSensor.getState();

        flywheel0.setVelocity(flywheelSpeed);
        if (flywheel1Enabled) {
            flywheel1.setMotorEnable();
            flywheel1.setPower(flywheel0.getPower());
        } else {flywheel1.setMotorDisable();}

        //if (flywheel0.getVelocity() >= flywheelSpeed-21 && flywheel0.getVelocity() <= flywheelSpeed+21) {flywheel1Enabled = false;} else {flywheel1Enabled = true;}

        panelsTelemetry.addData("FlywheelSpeed",flywheel0.getVelocity());
    }


}
