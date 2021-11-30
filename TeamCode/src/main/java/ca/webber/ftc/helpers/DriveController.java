package ca.webber.ftc.helpers;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

public class DriveController {

    private DcMotor leftFront, rightFront, leftBack, rightBack;
    private Servo openBucket, tiltBucket;
    private DcMotor boxArm;
    private final int[] OPEN_BUCKET_BOUNDARIES = {-1, 1}, TILT_BUCKET_BOUNDARIES = {-1, 1};
    private final double[] BOX_ARM_BOUNDARIES = {0, 2000};

    private boolean isBucketOpen;

    public DriveController (
            DcMotor leftFront,
            DcMotor rightFront,
            DcMotor leftBack,
            DcMotor rightBack,
            Servo openBucket,
            Servo tiltBucket,
            DcMotor boxArm
    ) {

        this.leftFront = leftFront;
        this.rightFront = rightFront;
        this.leftBack = leftBack;
        this.rightBack = rightBack;

        this.leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        this.rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        this.leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        this.rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        this.openBucket = openBucket;
        this.tiltBucket = tiltBucket;
        this.boxArm = boxArm;

        this.openBucket.setPosition(OPEN_BUCKET_BOUNDARIES[0]);
        this.tiltBucket.setPosition(OPEN_BUCKET_BOUNDARIES[0]);

        this.isBucketOpen = true;
    }

    public void update (double x, double y, double r) {
        leftFront.setPower(activationFunction(getTransXPrime(x, y) + r));
        rightFront.setPower(activationFunction(getTransYPrime(x, y) - r));
        leftBack.setPower(activationFunction(getTransYPrime(x, y) + r));
        rightBack.setPower(activationFunction(getTransXPrime(x, y) - r));
    }

    private double getTransXPrime (double x, double y) {
        return ((x + y) / Math.sqrt(2));
    }

    private double getTransYPrime (double x, double y) {
        return ((y - x) / Math.sqrt(2));
    }

    private double activationFunction (double a) {
//        if (a <= 1 && a >= -1) return a;
//        else if (a > 1) return 1;
//        else return -1;

//         optionally
         return Math.tanh(a);
    }

    private void moveArm(double y) {
        if (y < 0) boxArm.setPower((boxArm.getCurrentPosition() <= BOX_ARM_BOUNDARIES[0]) ? 0: y);
        else boxArm.setPower((boxArm.getCurrentPosition() >= BOX_ARM_BOUNDARIES[1]) ? 0: y);
    }

    private void toggleOpenBucket() {
        isBucketOpen = !isBucketOpen;
        openBucket.setPosition(OPEN_BUCKET_BOUNDARIES[isBucketOpen ? 0: 1]);
        // open is the first index
    }

    private double sigmoid(double x) {
        return 1 / (1 + Math.pow(Math.E, -x));
    }

    private void updateBucketTilt(double rot) {
        double currPos = tiltBucket.getPosition();
        double targetPos = currPos + (0.001 * (sigmoid(rot) - 0.5));
        // This is really a questionable thought. Forget the extra vars, I thought I had an idea
        tiltBucket.setPosition(targetPos);
    }

}
