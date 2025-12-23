package org.firstinspires.ftc.teamcode.resources;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class robotCfg {
    public static double WHEEL_SPEED = .25; //Maximum wheel speed (used to slow down the robot, or not)

    @Configurable
    public static class launcher_intake {

        //region launcher
        public static int flywheelMaxSpeed = 2700;
        public static double assistAmount = 0;
        public static double assistSpeed = 0;
        //endregion

        //region intake
        public static double intakeSpeed = 1;
        public static double intakeTime = 0; 

        //endregion
    }

    @Configurable
    public static class flywheelPID {

        public static double P = .89;
        public static double I = .222;
        public  static double D = .01;
    }

}
