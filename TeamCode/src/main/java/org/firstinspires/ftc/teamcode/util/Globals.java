package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class Globals {

    public static HardwareMap hardwareMap;

    public static boolean LIMITS = true;

    public static boolean IS_AUTO = false;

    public static COLORS ALLIANCE = COLORS.BLUE;

    public enum COLORS {
        RED,
        BLUE,
        YELLOW,
        NONE
    }

}
