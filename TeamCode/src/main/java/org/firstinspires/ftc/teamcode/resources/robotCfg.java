package org.firstinspires.ftc.teamcode.resources;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class robotCfg {
    public static double WHEEL_SPEED = .25; //Maximum wheel speed (used to slow down the robot, or not)

    @Configurable
    public static class turret {
        public static int flywheelMaxSpeed = 2700;
        public static boolean flywheel1Enabled = true;

        public static int motorTpr = 537;
        public static int aziRatio = 10;
    }

    @Configurable
    public static class intake {
        public static double intakeSpeed = 1;
    }

    @Configurable
    public static class indexer {
        public static double[] kickerCalibration = {0.45,0,0};
        public static String[] inventory = {"e","e","e"};
        public static double downTime = .5;
    }

    @Configurable
    public static class DEBUG_TOOLS {
        public static boolean _DEBUG_ = false;
        public static double azimuth = 0;
        public static double elevation = 0;
    }

    @Configurable
    public static class flywheelPID {

        public static double P = .89;
        public static double I = .222;
        public  static double D = .01;
    }

}
