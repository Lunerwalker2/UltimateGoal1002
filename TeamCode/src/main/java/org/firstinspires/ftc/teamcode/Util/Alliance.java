package org.firstinspires.ftc.teamcode.Util;

public enum Alliance {

    BLUE(true, false),
    RED(true, false),
    REMOTE_AUTO(true, true),
    REMOTE_TELEOP(false, true),
    TELEOP(false, false);


    public final boolean isAuto;
    public final boolean isRemote;

    Alliance(boolean isAuto, boolean isRemote){
        this.isAuto = isAuto;
        this.isRemote = isRemote;
    }

}
