package org.firstinspires.ftc.teamcode.resources;

import android.graphics.Color;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Arrays;
import java.util.Timer;
import java.util.concurrent.CompletableFuture;

import org.firstinspires.ftc.vision.opencv.ColorRange;

@Configurable
public class indexer {

    //region kickers
    Servo kCh0;
    Servo kCh1;
    Servo kCh2;
    //endregion

    //region colorSensors
    ColorRangeSensor CsCh0;
    ColorRangeSensor CsCh1;
    ColorRangeSensor CsCh2;
    //endregion

    TelemetryManager panelsTelemetry;

    public static double[] kickerCalibration = {0,0,0};
    public static String[] inventory = {"e","e","e"};
    public static double downTime = 1;


    public indexer(HardwareMap hwMp, TelemetryManager pTel) {
        kCh0 = hwMp.get(Servo.class,"ch0_kicker");
        kCh1 = hwMp.get(Servo.class,"ch1_kicker");
        kCh2 = hwMp.get(Servo.class,"ch2_kicker");

        //CsCh0 = hwMp.get(ColorRangeSensor.class,"ch0_color");
        //CsCh1 = hwMp.get(ColorRangeSensor.class,"ch1_color");
        //CsCh2 = hwMp.get(ColorRangeSensor.class,"ch2_color");

        panelsTelemetry = pTel;
    }

    private String getBall(NormalizedRGBA det) {
        return "e";
    }

    private int findColor(String c) {
        return Arrays.binarySearch(inventory,c);
    }

    private void reqChamber(int ch) {
        ElapsedTime downTimer = new ElapsedTime();
        downTimer.reset();
        switch (ch) {
            case 0:
                kCh0.setPosition(kickerCalibration[0]);
                //inventory[0] = "e";
            case 1:
                kCh1.setPosition(kickerCalibration[1]);
                //inventory[1] = "e";
            case 2:
                kCh2.setPosition(kickerCalibration[2]);
                //inventory[2] = "e";
        }
        while (downTimer.seconds()<=downTime) {
            panelsTelemetry.addData("waiting","ch"+ch);
        }
        kCh0.setPosition(0);
        kCh1.setPosition(0);
        kCh2.setPosition(0);

    }

    private void holdModu(double time) {
        ElapsedTime timer = new ElapsedTime();
        while (timer.seconds()<=time) {
            panelsTelemetry.addData("waiting","have to");
        }
    }

    public void updtInventory() {
        inventory[0] = getBall(CsCh0.getNormalizedColors());
        inventory[1] = getBall(CsCh1.getNormalizedColors());
        inventory[2] = getBall(CsCh2.getNormalizedColors());
    }

    public void reqBall(String req) {
        CompletableFuture.runAsync(() -> {
            for (int i = 0; i < req.length(); i++) {
                switch (req.substring(i, i + 1)) {
                    case "g":
                        reqChamber(findColor("g"));
                        holdModu(downTime*2);
                    case "p":
                        reqChamber(findColor("p"));
                        holdModu(downTime*2);
                }
            }
        });
    }
    public void reqCount(int c) {
        int curCh = 0;
        for (String ch :inventory ) {
            if (ch.equals("e")) {panelsTelemetry.addData(String.format("Channel %d empty",curCh),"");} else {reqChamber(curCh);}
            curCh++;
        }
    }
    public void calbArms() {
        
    }

}
