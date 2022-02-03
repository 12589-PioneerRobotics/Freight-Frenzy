package org.firstinspires.ftc.teamcode;

import android.graphics.Point;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import org.opencv.core.Scalar;

public class FieldConstants {

    public enum ShippingElementPosition{
        LEFT,
        CENTER,
        RIGHT
    }

    public static int maximumX = 72;
    public static int maximumY = 72;
    public static int minimumX = -72;
    public static int minimumY = -72;

    // All coordinate values should be mapped in inches(unfortunately) from their position relative to the origin

    public static Pose2d redCarouselStart = new Pose2d(-36.75, -58, Math.toRadians(90));
    public static Pose2d redWarehouseStart = new Pose2d(-68, -20, Math.toRadians(0));
    public static Pose2d blueCarouselStart = new Pose2d(68, 36, Math.toRadians(180));
    public static Pose2d blueWarehouseStart = new Pose2d(68, -20, Math.toRadians(180));

    public static Pose2d redCarousel = new Pose2d(-61, -55, Math.toRadians(40));
    public static Vector2d blueCarousel = new Vector2d(0, 0);

    public static Vector2d redDepot = new Vector2d(-64, -33);
    public static Vector2d blueDepot = new Vector2d(34.5, 57.5);

    public static Vector2d redWarehouse = new Vector2d(48, -40);
    public static Vector2d blueWarehouse = new Vector2d(46, -46);

    public static Pose2d transitionPoint = new Pose2d(-60, -17);
    public static Vector2d redShippingHub = new Vector2d(-11.5, -23);
    public static Vector2d blueShippingHub = new Vector2d(23, 11.5);
    public static Vector2d transitionPoint2 = new Vector2d(-62, -17);
    public static Vector2d transitionPoint3 = new Vector2d(-60, -49);


}
