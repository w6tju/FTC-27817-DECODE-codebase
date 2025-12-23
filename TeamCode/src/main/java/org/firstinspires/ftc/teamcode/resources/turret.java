package org.firstinspires.ftc.teamcode.resources;

import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

public class turret {
    DcMotorEx azimuth;
    Servo elevation;

    DcMotorEx flywheel0;
    DcMotorEx flywheel1;

    PanelsTelemetry panelsTelemetry;

    public turret(HardwareMap hwMp, PanelsTelemetry ptel) {
        azimuth = hwMp.get(DcMotorEx.class,"azimuth");
        elevation = hwMp.get(Servo.class,"elev");
        flywheel0 = hwMp.get(DcMotorEx.class,"flywheel0");
        flywheel1 = hwMp.get(DcMotorEx.class,"flywheel1");

        panelsTelemetry = ptel;
    }

    //automatic
    public void aimToPos(Pose botPos, Pose3D aimTo) {
        
    }

    //manual
    public void adjustTurret(double elevation, double azimuth,int flywheel) {

    }


}
