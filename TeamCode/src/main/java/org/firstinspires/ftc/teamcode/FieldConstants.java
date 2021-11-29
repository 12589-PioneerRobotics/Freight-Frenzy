package org.firstinspires.ftc.teamcode;

import android.graphics.Point;

import org.opencv.core.Scalar;

public class FieldConstants {

    public enum ShippingElementPosition{
        LEFT,
        CENTER,
        RIGHT
    }

    public static double ticksToInch = 0; // To be tested

    public static Point fieldMaximum = new Point(0, 0); // Dependent on the RoadRunner Library

    // All coordinate values should be mapped in inches(unfortunately) from their position relative to the origin

    public static Point blueLeftElement1 = new Point(0, 0);
    public static Point blueCenterElement1 = new Point(0, 0);
    public static Point blueRightElement1 = new Point(0, 0);
    public static Point blueLeftElement2 = new Point(0, 0);
    public static Point blueCenterElement2 = new Point(0, 0);
    public static Point blueRightElement2 = new Point(0, 0);

    public static Point redLeftElement1 = new Point(0, 0);
    public static Point redCenterElement1 = new Point(0, 0);
    public static Point redRightElement1 = new Point(0, 0);
    public static Point redLeftElement2 = new Point(0, 0);
    public static Point redCenterElement2 = new Point(0, 0);
    public static Point redRightElement2 = new Point(0, 0);

    public static Point blueBeacon = new Point(0, 0);
    public static Point redBeacon = new Point(0, 0);

    public static Point blueCarousel = new Point(0, 0);
    public static Point redCarousel = new Point(0, 0);

    public static Point mutualBeacon = new Point(0, 0);

    public static Scalar elementColor = new Scalar(0, 0, 0); // To be determined



}
