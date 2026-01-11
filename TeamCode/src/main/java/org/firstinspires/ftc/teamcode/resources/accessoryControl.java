package org.firstinspires.ftc.teamcode.resources;

import static org.firstinspires.ftc.teamcode.resources.robotCfg.*;
import static org.firstinspires.ftc.teamcode.resources.robotCfg.DEBUG_TOOLS.*;
import static org.firstinspires.ftc.teamcode.resources.robotCfg.intake.*;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.concurrent.CompletableFuture;

@Configurable
public class accessoryControl {

    private static final Logger log = LoggerFactory.getLogger(accessoryControl.class);
    public static boolean _DEBUG_ = false;
    public static String motif = "gpp";

    //Physical Objects
    public DcMotorEx flywheel0;
    public  DcMotorEx flywheel1;
    public DcMotorEx intake;
    DcMotorEx azimuth;

    boolean driverRumble = false;
    public static int flywheelSpeed = 1300;
    public static boolean flywheelOn = false;
    Gamepad driveController;

    turret tur;
    indexer index;

    private int toTPS(int rpm,int tpr) {return ((rpm/60)*tpr);}

    public accessoryControl(HardwareMap hardwareMap, TelemetryManager ptel, boolean AUTO) {
        //region Hardware
        intake = hardwareMap.get(DcMotorEx.class,"intake");
        //endregion

        //region Directions
        intake.setDirection(Direction.FORWARD);
        //endregion

        //region Zero power
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //endregion

        //region modules
        tur = new turret(hardwareMap,ptel);
        index = new indexer(hardwareMap, ptel);
        //endregion
    }

    public void atStart() {

    }
    public void RunAccessory(Gamepad gamepad1, Gamepad driver) {
        if (!   _DEBUG_) {
            driveController = driver;

            if (gamepad1.triangleWasPressed()) {index.reqBall(motif);}
            if (gamepad1.rightBumperWasPressed()) { if (intake.getPower() == intakeSpeed) {Set_intake(0);} else {Set_intake(intakeSpeed);}}
            if (gamepad1.leftBumperWasPressed()) {flywheelOn = !flywheelOn;}
            if (gamepad1.dpadDownWasPressed()){flywheelSpeed *= -1;}
            if (gamepad1.dpadUpWasPressed()) {index.reqChamber(2);}

        } else {Debug_Mode();}
        Run_Motors(); // we call Run_Motors here so that you dont have to in the op-mode loop
    }
    public void Run_Motors() {
        //Handles motor running, useful for AUTO when there is no controller input\\
        if (flywheelOn) {tur.flywheelSpeed = flywheelSpeed;} else {tur.flywheelSpeed = 0;}
        tur.passiveRun();
        index.passiveRun();
    }

    public void Set_intake(double Intake_speed) {
        CompletableFuture.runAsync(() -> {
           intake.setPower(Intake_speed);
        });
    }


    public void Debug_Mode() {
        tur.adjustTurret(DEBUG_TOOLS.elevation,DEBUG_TOOLS.azimuth,robotCfg.turret.flywheelMaxSpeed);
        index.calbArms();
    }
}