package org.firstinspires.ftc.teamcode.CustomStuff.yes;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import java.util.ArrayList;

public class PropDetectionPipeline1 implements VisionProcessor {

    public static int hsvLow1 = 30;
    public static int hsvLow2 = 170;
    public static int hsvLow3 = 30;
    public static int hsvHigh1 = 180;
    public static int hsvHigh2 = 255;
    public static int hsvHigh3 = 255;
    public static int numContours = 0;

    //private final double X_MIN_RIGHT, Y_MIN_RIGHT, X_MIN_LEFT, Y_MIN_LEFT, X_MIN_MID, Y_MIN_MID, X_MAX_RIGHT, Y_MAX_RIGHT, X_MAX_LEFT, Y_MAX_LEFT, X_MAX_MID, Y_MAX_MID;

    public static double largestContourArea = 0;
    public static double largestContourX, largestContourY;
    private MatOfPoint largestContour;
    public int location;

    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }

    @Override
    public Object processFrame(Mat input, long captureTimeNanos) {
        // Make a working copy of the input matrix in HSV
        Mat mat = new Mat();
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        Mat edges = new Mat();
        Core.inRange(mat, new Scalar(hsvLow1, hsvLow2, hsvLow3), new Scalar(hsvHigh1, hsvHigh2, hsvHigh3), edges);
        edges.copyTo(input);
        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }
}
