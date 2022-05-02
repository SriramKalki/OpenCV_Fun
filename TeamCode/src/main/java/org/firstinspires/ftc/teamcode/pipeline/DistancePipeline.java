package org.firstinspires.ftc.teamcode.pipeline;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class DistancePipeline extends OpenCvPipeline {
    //creates 3 matrices

    Mat mat = new Mat();
    Mat output = new Mat();

    //enum for TSE position

    //TODO find the threshold value- the minimum upper/lowerValue needed to ensure that there is a TSE

    //For camera stream coloring purposes
    final Scalar green = new Scalar(0, 255, 0);
    final Scalar red = new Scalar(255, 0, 0);
    final Scalar blue = new Scalar(0, 0, 255);

    Scalar lowHSV = new Scalar(23, 50, 70);
    Scalar highHSV = new Scalar(32, 255, 255);

    final double INCHES_PER_PIXEL_Y = Math.sqrt(436)/240.0;
    final double WEBCAM_ANGLE_X = 41.186 * 2;
    final double WEBCAM_ANGLE_Y = 73.301;
    //Telemetry
    Telemetry telemetry;

    //CONSTRUCTOR
    public DistancePipeline(Telemetry t) {
        //setting telemetry
        telemetry = t;

    }
    public enum Location{
        LEFT,MIDDLE,RIGHT
    }

    @Override
    public Mat processFrame(Mat input) {

        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGBA2RGB);
        final int WIDTH = mat.width();
        final int HEIGHT = mat.height();
        output = input;
        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_RGB2HSV);
        Core.inRange(mat, lowHSV, highHSV, mat);

        List<MatOfPoint> countersList = new ArrayList<>();
        Imgproc.findContours(mat, countersList, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.drawContours(mat, countersList,0, new Scalar(255,0,0));
        Imgproc.drawContours(output, countersList,0, new Scalar(255,0,0));
        Rect theRealDucky = new Rect(new Point(0,0), new Point(1,1));
        for (MatOfPoint countor : countersList)
        {

            Rect rect = Imgproc.boundingRect(countor);
            if (rect.area() > theRealDucky.area()) {
                theRealDucky = rect;
            }

        }
        String forward = "";
        String strafe = "";
        String bearing = "";
        Imgproc.rectangle(output, theRealDucky, new Scalar(0,0,0));
        if(theRealDucky.area() <= 100){
            return output;
        }


        Point midpoint = new Point((theRealDucky.x + theRealDucky.width/2.0),(theRealDucky.y + theRealDucky.height/2.0));

        //midpoint.y * sqrt{436}/240 gives us the straight line distance from the webcam to object
        double straightLineDistanceY = INCHES_PER_PIXEL_Y * (240- midpoint.y);
        double horizontalDistanceY = straightLineDistanceY * Math.sin(Math.toRadians(WEBCAM_ANGLE_Y)) - 3.2;

        forward = Math.round(horizontalDistanceY) + " inches forward";
        Imgproc.putText(output,forward,new Point(theRealDucky.x - 60, theRealDucky.y),1,0.8,green);
        telemetry.addData("Distance Y: ", horizontalDistanceY);

        //reference: 8 in horDistance when 14 is the width
        double horizontalDistanceX = 2 * (0.5* mat.width() - midpoint.x) * horizontalDistanceY * Math.tan(WEBCAM_ANGLE_X) / mat.width();
        strafe = Math.round(horizontalDistanceX) + " inches right ";
        Imgproc.putText(output,strafe,midpoint,1,0.8,red);
        telemetry.addData("Distance X: ", horizontalDistanceX);

        double angle = Math.toDegrees(Math.atan(horizontalDistanceX/horizontalDistanceY));
        bearing = Math.round(angle) + " bearing (degrees)";
        Imgproc.putText(output,bearing,new Point((theRealDucky.x+ theRealDucky.width),(theRealDucky.y+ theRealDucky.height)),1,0.8,blue);
        telemetry.addData("Estimated Turn: ", angle);

        telemetry.update();
        return output;

    }
}