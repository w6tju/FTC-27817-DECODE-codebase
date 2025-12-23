package org.firstinspires.ftc.teamcode.resources;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class chassisKinematics {
    public enum controlRelativity {
        Robot,
        Field
    }
    public driveKinematicController getKinematicsController(HardwareMap hardwareMap, controlRelativity controlRelativity) {
        driveKinematicController driveController;
        driveController = new driveKinematicController();
        driveController.init(hardwareMap);
        return driveController;
    }
}
