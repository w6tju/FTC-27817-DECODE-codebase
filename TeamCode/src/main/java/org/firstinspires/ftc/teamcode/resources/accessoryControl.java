package org.firstinspires.ftc.teamcode.resources;

import static org.firstinspires.ftc.teamcode.resources.robotCfg.*;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
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
    public static String motif = "gpp";

    //Physical Objects
    public DcMotorEx flywheel0;
    public  DcMotorEx flywheel1;
    public DcMotorEx intake;
    DcMotorEx azimuth;

    boolean driverRumble = false;
    int flywheelSpeed = 0;
    Gamepad driveController;

    indexer index;

    public accessoryControl(HardwareMap hardwareMap, boolean AUTO) {
        //region Hardware
        flywheel0 = (DcMotorEx)hardwareMap.get(DcMotor.class,"flywheel0");
        flywheel1 = (DcMotorEx)hardwareMap.get(DcMotor.class,"flywheel1");
        intake = (DcMotorEx)hardwareMap.get(DcMotor.class,"intake");
        azimuth = (DcMotorEx)hardwareMap.get(DcMotor.class,"azimuthControl");
        //endregion

        //region Directions
        flywheel0.setDirection(Direction.FORWARD);
        flywheel1.setDirection(Direction.REVERSE);

       intake.setDirection(Direction.FORWARD);
        //endregion

        //region Zero power
        flywheel0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheel1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        //endregion

        //region PID
        flywheel0.setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,new PIDCoefficients(robotCfg.flywheelPID.P, robotCfg.flywheelPID.I, robotCfg.flywheelPID.D));
        flywheel1.setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,new PIDCoefficients(robotCfg.flywheelPID.P, robotCfg.flywheelPID.I, robotCfg.flywheelPID.D));
        //endregion

        //region modules
        index = new indexer(hardwareMap, PanelsTelemetry.INSTANCE.getTelemetry());
        //endregion
    }
    public void atStart() {

    }
    public void RunAccessory(Gamepad gamepad1, Gamepad driver) {
        driveController = driver;
        Run_Motors(); // we call Run_Motors here so that you dont have to in the op-mode loop

        if (gamepad1.squareWasPressed()) {Set_flywheel(1);}
        if (gamepad1.crossWasPressed()) {Set_intake(1);}
        if (gamepad1.circleWasPressed()) {Set_intake(0);}

        if (gamepad1.triangleWasPressed()) {index.reqBall(motif);}
    }
    public void Run_Motors() {
        //Handles motor running, useful for AUTO when there is no controller input\\
        flywheel0.setVelocity(flywheelSpeed);
        flywheel1.setVelocity(flywheelSpeed);
    }

    public void Set_intake(double Intake_speed) {
        CompletableFuture.runAsync(() -> {
           intake.setPower(Intake_speed);
        });
    }

    public void Set_flywheel(int Flywheel_speed) {
        CompletableFuture.runAsync(() -> {
            flywheelSpeed = Flywheel_speed;
        });
    }

    public void fire_Ball(int fireCount,int flywheel_Speed) {
        CompletableFuture.runAsync(() -> {
           for (int count=1; count<=fireCount; count++) {
               Set_flywheel(flywheel_Speed);
               while (flywheel0.getVelocity()<flywheel_Speed||flywheel1.getVelocity()<flywheel_Speed) {
                   log.debug("waiting on flywheel speed");
               }
               while (flywheel0.getVelocity()>flywheel_Speed-100||flywheel1.getVelocity()>flywheel_Speed-100) {
                   Set_intake(1);
               }
               Set_intake(0);
           }
        });
    }
    public void Debug_Mode() {

    }
}