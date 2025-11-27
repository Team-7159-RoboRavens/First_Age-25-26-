package org.firstinspires.ftc.teamcode;

// This class is just for storing data from the limelight
public class limelightData {
//The ID is meant to
    public static int aprilId;
    public static String aprilFamily;
    public static double aprilXDegrees;
    public static double aprilYDegrees;
    public static boolean accurate = false;
    public static boolean aiming = false;
    public static double distance;
    public static int pattern;



    //This simply changes all the variables at once.
    public static void setParams(int I, String F, double X, double Y) {
        aprilId = I;
        aprilFamily = F;
        aprilXDegrees = X;
        aprilYDegrees = Y;
    }

    //This should return the direction and scalar of the april tag, change to be the mean value of degrees.
    public static double[] directionToTag() {
        return new double[]{(aprilXDegrees) / 20, (aprilYDegrees) / 20};
    }
}
