package pedroPathing.opmode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="Mecanum Drive w/o Odometry (Robot centric)", group="Iterative Opmode")


public class Mecanum_drive extends OpMode{

    /*
     * The mecanum drivetrain involves four separate motors that spin in
     * different directions and different speeds to produce the desired
     * movement at the desired speed.
     */

    // declare and initialize four DcMotors.

    // We make them private so they can only be used in this class.
    private DcMotor front_left  = null;
    private DcMotor front_right = null;
    private DcMotor back_left   = null;
    private DcMotor back_right  = null;

    @Override
    public void init() {

        // Name strings must match up with the config on the Robot Controller
        front_left   = hardwareMap.get(DcMotor.class, "front_left");
        front_right  = hardwareMap.get(DcMotor.class, "front_right");
        back_left    = hardwareMap.get(DcMotor.class, "back_left");
        back_right   = hardwareMap.get(DcMotor.class, "back_right");

        // Correct reversing of motors (the ones of the ri
        front_right.setDirection(DcMotorSimple.Direction.REVERSE);
        back_right.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {

        // variable to set the powers of the 4 motors. They are dependent on joystick position
        double r = Math.hypot(-gamepad1.left_stick_x, gamepad1.left_stick_y);

        // variable to set the angle of the robot depending on the joystick input
        double robotAngle = Math.atan2(gamepad1.left_stick_y, -gamepad1.left_stick_x) - Math.PI / 4;
        // basically a bunch of math

        // Variable of turning
        double rightX = -gamepad1.right_stick_x; // Inverting the controls correctly

        // math to find the values to set the powers of the motors
        final double v1 = r * Math.cos(robotAngle) + rightX;
        final double v2 = r * Math.sin(robotAngle) - rightX;
        final double v3 = r * Math.sin(robotAngle) + rightX;
        final double v4 = r * Math.cos(robotAngle) - rightX;

        // finally setting the powers of the motors

        front_left.setPower(v1);
        front_right.setPower(v2);
        back_left.setPower(v3);
        back_right.setPower(v4);
    }
}