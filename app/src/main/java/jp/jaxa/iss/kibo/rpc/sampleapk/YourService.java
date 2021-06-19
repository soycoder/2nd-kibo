package jp.jaxa.iss.kibo.rpc.sampleapk;

import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;

import gov.nasa.arc.astrobee.Result;
import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;

import android.graphics.Bitmap;
import android.util.Log;

import com.google.zxing.BinaryBitmap;
import com.google.zxing.ChecksumException;
import com.google.zxing.FormatException;
import com.google.zxing.LuminanceSource;
import com.google.zxing.NotFoundException;
import com.google.zxing.RGBLuminanceSource;
import com.google.zxing.common.HybridBinarizer;
import com.google.zxing.qrcode.QRCodeReader;

import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import java.util.ArrayList;
import java.util.List;


/**
 * Class meant to handle commands from the Ground Data System and execute them in Astrobee
 */

public class YourService extends KiboRpcService {

    public Mat processedImg;

    public Mat threshImg;

    public Mat sharpenImg;
    private int kernelSize = 3;
    private  Mat element = Imgproc.getStructuringElement(Imgproc.CV_SHAPE_RECT,
            new Size(2 * kernelSize + 1, 2 * kernelSize + 1), new org.opencv.core.Point(kernelSize, kernelSize));
    public Mat cropped_img;
    public Mat warped_img;
    public org.opencv.core.Point text_position;
    public Rect target_rect;


    //    public Mat grayImg;
    //    public Mat processedCircleImg;

    @Override
    protected void runPlan1(){
        // astrobee is undocked and the mission starts
        api.startMission();

        // TODO: Move astrobee to point-A
        Log.d("Move[status]:","point-A | starting...");
        moveToWrapper(11.21, -9.8, 4.79, 0, 0, -0.707, 0.707);
        Log.d("Move[status]:","point-A | finished...");

        // TODO: Read QR
        Mat image = api.getMatNavCam();
        String content = null;
        try {
            content = readQR(image);
        } catch (ChecksumException e) {
            e.printStackTrace();
        } catch (FormatException e) {
            e.printStackTrace();
        } catch (NotFoundException e) {
            e.printStackTrace();
        }

        Log.d("QR[data]:", content);

        // Fixed pattern2 {"p":2,"x":11.09,"y":-9.80,"z":5.24}

         api.sendDiscoveredQR(content);

        // TODO: Move astrobee to point-A'
        Log.d("Move[status]:","point-A\' | starting...");
        moveToWrapper(11.09, -9.8, 5.24, 0, 0, -0.707, 0.707);
        Log.d("Move[status]:","point-A\' | finished...");

        // TODO: Rotate

        // TODO: Irradiate the laser
        api.laserControl(true);
        Log.d("Laser[status]:","Irradiating...");

        // take snapshots
        // api.takeSnapshot();

        // irradiate the laser
        api.laserControl(true);
        Log.d("Laser[status]:","finished...");


        // TODO: Move astrobee from point-A' to aside KOZ_2
        moveToWrapper(10.40, -9.806, 4.293, 0, 0, -0.707, 0.707);
        moveToWrapper(10.40, -9.0, 4.293, 0, 0, -0.707, 0.707);

        // TODO: Move astrobee to point-B'
        // Log.d("Move To Point","-B");
        // Log.d(">","===========================================================");
        moveToWrapper(10.6, -8.0, 4.5, 0, 0, -0.707, 0.707);


        // Send mission completion
        api.reportMissionCompletion();
    }

    @Override
    protected void runPlan2(){
        // write here your plan 2
    }

    @Override
    protected void runPlan3(){
        // write here your plan 3
    }

    // ! Moving
    private void moveToWrapper(double pos_x, double pos_y, double pos_z,
                               double qua_x, double qua_y, double qua_z,
                               double qua_w){

        final int LOOP_MAX = 3;
        final Point point = new Point(pos_x, pos_y, pos_z);
        final Quaternion quaternion = new Quaternion((float)qua_x, (float)qua_y,
                                                     (float)qua_z, (float)qua_w);

        Result result = api.moveTo(point, quaternion, true);

        int loopCounter = 0;
        while(!result.hasSucceeded() || loopCounter < LOOP_MAX){
            result = api.moveTo(point, quaternion, true);
            ++loopCounter;
        }
    }

    private Quaternion eulerToQuaternion(double yaw_degree, double pitch_degree, double roll_degree){

        double yaw = Math.toRadians(yaw_degree); //radian = degree*PI/180
        double pitch = Math.toRadians(pitch_degree);
        double roll = Math.toRadians(roll_degree);

        double cy = Math.cos(yaw * 0.5);
        double sy = Math.sin(yaw * 0.5);
        double cp = Math.cos(pitch * 0.5);
        double sp = Math.sin(pitch * 0.5);
        double cr = Math.cos(roll * 0.5);
        double sr = Math.sin(roll * 0.5);

        double qx = sr * cp * cy - cr * sp * sy;
        double qy = cr * sp * cy + sr * cp * sy;
        double qz = cr * cp * sy - sr * sp * cy;
        double qw = cr * cp * cy + sr * sp * sy;

        Quaternion quaternion = new Quaternion((float)qx, (float)qy, (float)qz, (float)qw);

        return quaternion;
    }

    // ! Moving <End>

    // ! QR Code Method
    public String readQR(Mat qr_image) throws ChecksumException, FormatException, NotFoundException {
        findRectContours(qr_image);
        String qr_info = decodeQRCode(sharpenImg);
        return qr_info;
    }
    // ! QR Code Method <End>

    // ! ImageProcessingQR
    private org.opencv.core.Point[] sortingPoints(MatOfPoint pts, int x, int y) {
        org.opencv.core.Point[] sortedPoints = new org.opencv.core.Point[4];
        double data[];
        for (int i = 0; i < pts.size().height; i++) {
            data = pts.get(i, 0);
            double datax = data[0];
            double datay = data[1];
            // 0-------1
            // | |
            // | x,y |
            // | |
            // 2-------3
            if (datax < x && datay < y) {
                sortedPoints[0] = new org.opencv.core.Point(datax, datay);
            } else if (datax > x && datay < y) {
                sortedPoints[1] = new org.opencv.core.Point(datax, datay);
            } else if (datax < x && datay > y) {
                sortedPoints[2] = new org.opencv.core.Point(datax, datay);
            } else if (datax > x && datay > y) {
                sortedPoints[3] = new org.opencv.core.Point(datax, datay);
            }
        }
        return sortedPoints;

    }

    private Mat sharpeningImg(Mat src) {
        Mat dst = new Mat(src.rows(), src.cols(), src.type());
        Imgproc.medianBlur(src, dst, 7);
        Core.subtract(src, dst, dst);
        Core.add(dst, src, dst);

        return dst;
    }

    private Mat thresholding(Mat img) {
        Mat gray = new Mat(img.rows(), img.cols(), img.type());
        Imgproc.cvtColor(img, gray, Imgproc.COLOR_BGR2GRAY);
        Mat binaryImg = new Mat(img.rows(), img.cols(), img.type(), new Scalar(0));
        Imgproc.threshold(gray, binaryImg, 250, 255, Imgproc.THRESH_BINARY);
        Imgproc.erode(binaryImg, binaryImg, element);
        return binaryImg;
    }


    public void findRectContours(Mat img) {

        processedImg = new Mat(img.rows(), img.cols(), img.type());
        img.copyTo(processedImg);
        Mat binImg = thresholding(img);

        threshImg = new Mat(img.rows(), img.cols(), img.type(), new Scalar(0));

        binImg.copyTo(threshImg);

        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchey = new Mat();
        Imgproc.findContours(binImg, contours, hierarchey, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        for (int i = 0; i < contours.size(); i++) {
            Scalar color = new Scalar(0, 255.0, 0);
            // Drawing Contours
            if (hierarchey.get(0, i)[2] == -1.0) {
                MatOfPoint2f ct2f = new MatOfPoint2f(contours.get(i).toArray());
                MatOfPoint2f approxCurve = new MatOfPoint2f();
                Moments moment = Imgproc.moments(ct2f);

                int x = (int) (moment.get_m10() / moment.get_m00());
                int y = (int) (moment.get_m01() / moment.get_m00());

                double approxDistance = Imgproc.arcLength(ct2f, true) * 0.1;
                Imgproc.approxPolyDP(ct2f, approxCurve, approxDistance, true);
                MatOfPoint points = new MatOfPoint(approxCurve.toArray());
                if (points.size().height == 4.0) {
                    target_rect = Imgproc.boundingRect(points);

                    text_position = new org.opencv.core.Point(target_rect.x, target_rect.y);
                    cropped_img = new Mat();
                    cropped_img = img.submat(target_rect);

                    org.opencv.core.Point[] sorted_pts = sortingPoints(points, x, y);

                    // TODO: Draw 4 circles at each corner
                    // for (int j = 0; j < sorted_pts.length; j++) {
                    // 	Point p = new Point(sorted_pts[j].x, sorted_pts[j].y);
                    // 	Imgproc.circle(processedImg, p, 5, new Scalar(255, 0, 0), -1);
                    // }

                    MatOfPoint2f src_pts = new MatOfPoint2f();
                    src_pts.fromArray(sorted_pts);

                    double w1 = Math.sqrt(Math.pow((sorted_pts[1].x - sorted_pts[0].x), 2)
                            + Math.pow((sorted_pts[1].y - sorted_pts[0].y), 2));
                    double w2 = Math.sqrt(Math.pow((sorted_pts[3].x - sorted_pts[2].x), 2)
                            + Math.pow((sorted_pts[3].y - sorted_pts[2].y), 2));

                    double h1 = Math.sqrt(Math.pow((sorted_pts[1].x - sorted_pts[3].x), 2)
                            + Math.pow((sorted_pts[1].y - sorted_pts[3].y), 2));
                    double h2 = Math.sqrt(Math.pow((sorted_pts[0].x - sorted_pts[2].x), 2)
                            + Math.pow((sorted_pts[0].y - sorted_pts[2].y), 2));

                    double max_w = Math.max(w1, w2);
                    double max_h = Math.max(h1, h2);

                    MatOfPoint2f dst_pts = new MatOfPoint2f(new org.opencv.core.Point(0, 0), new org.opencv.core.Point(max_w - 1, 0),
                            new org.opencv.core.Point(0, max_h - 1), new org.opencv.core.Point(max_w - 1, max_h - 1));
                    Mat perspective_tf = Imgproc.getPerspectiveTransform(src_pts, dst_pts);

                    warped_img = new Mat();
                    Imgproc.warpPerspective(img, warped_img, perspective_tf, new Size(max_w, max_h),
                            Imgproc.INTER_LINEAR);

                    sharpenImg = new Mat();
                    sharpenImg = sharpeningImg(warped_img);
                    Log.d("QR[size]", " "+sharpenImg.size());
                }
            }
        }
    }
    // ! ImageProcessingQR <End>

    // ! zxingQRreader


    public String decodeQRCode(Mat img) throws ChecksumException, FormatException {

        Bitmap bMap2 = Bitmap.createBitmap(img.width(),img.height(), Bitmap.Config.ARGB_8888);
        Utils.matToBitmap(img, bMap2);
        int[] intArray2 = new int[bMap2.getWidth()*bMap2.getHeight()];
        bMap2.getPixels(intArray2, 0, bMap2.getWidth(), 0, 0, bMap2.getWidth(), bMap2.getHeight());
        LuminanceSource source2 = new RGBLuminanceSource(bMap2.getWidth(), bMap2.getHeight(),intArray2);

        BinaryBitmap bitmap2 = new BinaryBitmap(new HybridBinarizer(source2));

        try {
            com.google.zxing.Result result = new QRCodeReader().decode(bitmap2);
            return result.getText();
        } catch (NotFoundException e) {
            System.out.println("There is no QR code in the image");
            return null;
        }
    }
    // ! zxingQRreader <End>
}

