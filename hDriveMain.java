package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Holonomic OpMode WORKING", group="Iterative Opmode")
//@Disabled
public class hDriveMain extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor  FrontRightMotor, FrontLeftMotor, BackRightMotor, BackLeftMotor;
    private DcMotorEx liftMotor, liftMotor2;
    //   private CRServo CMotor1, CMotor2, intakeServo;
    // private int[] liftPos = {0, 1800, 2800, 3900};
    private int[] liftPos = {0, 1660, 2800, 4000};
    private int[] coneStack = {240, 265, 325, 370};
    private int currentLiftPosition = 0;
    private int currentConePosition = 0;
    private int cLPInt = 0;
    private Servo intakeServo;
    HolonomicDrive holonomicDrive;
    boolean YIsPressed = false;
    boolean XIsPressed = false;
    boolean BIsPressed = false;
    boolean AIsPressed = false;
    boolean RBIsPressed = false;
    boolean LBIsPressed = false;
    boolean DPADLeftIsPressed = false;
    boolean DPADRightIsPressed = false;
    boolean LTIsPressed = false;
    boolean RTIsPressed = false;
    boolean BackIsPressed = false;
    boolean closed = false;
    boolean up = false;
    boolean forward = false;
    boolean B2IsPressed = false;
    boolean LB2IsPressed = false;
    boolean RB2IsPressed = false;
    boolean X2IsPressed = false;
    private boolean Y2IsPressed = false;
    private boolean A2IsPressed = false;
    boolean back2IsPressed = false;
    private boolean RJoyEngaged = false;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        FrontRightMotor  = hardwareMap.get(DcMotor.class, "frontRight");
        FrontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeft");
        BackRightMotor  = hardwareMap.get(DcMotor.class, "backRight");
        BackLeftMotor = hardwareMap.get(DcMotor.class, "backLeft");
        liftMotor = hardwareMap.get(DcMotorEx.class, "LiftMotor1");
        liftMotor2 = hardwareMap.get(DcMotorEx.class, "LiftMotor2");
        intakeServo = hardwareMap.get(Servo.class, "intakeServo");

        holonomicDrive = new HolonomicDrive(FrontRightMotor, FrontLeftMotor, BackRightMotor, BackLeftMotor);


        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        liftMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//        liftMotor2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        liftMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        //LiftMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        liftMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        liftMotor2.setDirection(DcMotorSimple.Direction.FORWARD);}

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
        if(forward) {
            x = -gamepad1.left_stick_x;
            y = gamepad1.left_stick_y;
            z = gamepad1.right_stick_x;
        } else if(!forward) {
            x = gamepad1.left_stick_x;
            y = -gamepad1.left_stick_y;
            z = gamepad1.right_stick_x;
        }
        double RightJoyY2 = gamepad2.right_stick_y;

        boolean B_1Button = gamepad1.b;
        boolean A_1Button = gamepad1.a;
        boolean RB_1Button = gamepad1.right_bumper;
        boolean X_1Button = gamepad1.x;
        boolean Y_1Button = gamepad1.y;
        boolean LB_1Button = gamepad1.left_bumper;
        boolean DPL_1Button = gamepad1.dpad_left;
        boolean DPR_1Button = gamepad1.dpad_right;
        boolean backButton1 = gamepad1.back;

        boolean B_2Button = gamepad2.b;
        boolean A_2Button = gamepad2.a;
        boolean RB_2Button = gamepad2.right_bumper;
        boolean X_2Button = gamepad2.x;
        boolean Y_2Button = gamepad2.y;
        boolean LB_2Button = gamepad2.left_bumper;
        boolean backButton2 = gamepad1.back;

        // Open and close pick-point
        if(B_2Button && closed == false && B2IsPressed == false){
            closed = true;
            intakeServo.setPosition(0.0);
            B2IsPressed = true;
        }
        else if(B_2Button == false){
            B2IsPressed = false;
        }
        if(B_2Button && closed == true && B2IsPressed == false){
            closed = false;
            intakeServo.setPosition(0.165);
            B2IsPressed = true;
        }
        else if(B_2Button == false){
            B2IsPressed = false;
        }

        // sway arm
//        if(X_2Button == true && X2IsPressed == false && currentLiftPosition >=1) {
//            X2IsPressed = true;
//            if(up) {
//                up = false;
//                rotatorIntakeServo.setPosition(0.655);
//                forward = false;
//            } else {
//                up = true;
//                telemetry.addLine("this is zero rn");
//                telemetry.update();
//                rotatorIntakeServo.setPosition(0.25);
//                forward = true;
//            }
//        }
//        else if(X_2Button == false) {
//            X2IsPressed = false;
//        }



        // Make the lift go UP
        if(Y_2Button && !Y2IsPressed && currentLiftPosition < (liftPos.length - 1)){
            currentLiftPosition += 1;
            //   liftMotor.setTargetPosition(liftPos[currentLiftPosition]);


            //   liftMotor.setPower(0.9);
            liftMotor.setTargetPosition(liftPos[currentLiftPosition]);
            liftMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            liftMotor.setPower(0.9);
            liftMotor2.setTargetPosition(liftPos[currentLiftPosition]);
            liftMotor2.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            liftMotor2.setPower(0.9);
            Y2IsPressed = true;
            cLPInt = liftPos[currentLiftPosition];
        } else if(!Y_2Button) {
            Y2IsPressed = false;

        }

        //Make the lift go down
        if (A_2Button && !A2IsPressed && currentLiftPosition > 0){
            A2IsPressed = true;
            currentLiftPosition -= 1;
            liftMotor.setTargetPosition(liftPos[currentLiftPosition]);
            liftMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            liftMotor.setPower(0.9);
            liftMotor2.setTargetPosition(liftPos[currentLiftPosition]);
            liftMotor2.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            liftMotor2.setPower(0.9);
            cLPInt = liftPos[currentLiftPosition];
        } else if(!A_2Button) {
            A2IsPressed = false;
        }

        //custom lift override
        if (RightJoyY2<0 && !RJoyEngaged && currentLiftPosition > 0){
            RJoyEngaged = true;
            cLPInt += 100;
            liftMotor.setTargetPosition(cLPInt);
            liftMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            liftMotor.setPower(0.9);
            liftMotor2.setTargetPosition(cLPInt);
            liftMotor2.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            liftMotor2.setPower(0.9);
        } else if(RightJoyY2 == 0) {
            RJoyEngaged = false;
        }
        if (RightJoyY2>0 && !RJoyEngaged && currentLiftPosition > 0){
            RJoyEngaged = true;
            cLPInt -= 100;
            liftMotor.setTargetPosition(cLPInt);
            liftMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            liftMotor.setPower(0.9);
            liftMotor2.setTargetPosition(cLPInt);
            liftMotor2.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            liftMotor2.setPower(0.9);
        } else if(RightJoyY2 == 0) {
            RJoyEngaged = false;
        }

        // If the lift is positioned one place above where it needs to go next
        // drop the intake to take a cone from the stack

        if (RB_2Button && Y_2Button && !Y2IsPressed && currentLiftPosition > 0) {
            Y2IsPressed = true;
            currentConePosition = 0;
            liftMotor.setTargetPosition(coneStack[currentConePosition]);
            liftMotor2.setTargetPosition(coneStack[currentConePosition]);
            liftMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            liftMotor2.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            liftMotor.setPower(-0.9);
            liftMotor2.setPower(-0.9);
            intakeServo.setPosition(0.25);
        } else if (RB_2Button && X_2Button && !X2IsPressed && currentLiftPosition > 0) {
            X2IsPressed = true;
            currentConePosition = 1;
            liftMotor.setTargetPosition(coneStack[currentConePosition]);
            liftMotor2.setTargetPosition(coneStack[currentConePosition]);
            liftMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            liftMotor2.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            liftMotor.setPower(-0.9);
            liftMotor2.setPower(-0.9);
            intakeServo.setPosition(0.25);
        } else if (RB_2Button && A_2Button && !A2IsPressed && liftPos[currentLiftPosition] > 0) {
            A2IsPressed = true;
            currentConePosition = 2;
            liftMotor.setTargetPosition(coneStack[currentConePosition]);
            liftMotor2.setTargetPosition(coneStack[currentConePosition]);
            liftMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            liftMotor2.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            liftMotor.setPower(-0.9);
            liftMotor2.setPower(-0.9);
            intakeServo.setPosition(0.25);
        }   else if (RB_2Button && B_2Button && !B2IsPressed && currentLiftPosition > 0) {
            B2IsPressed = true;
            currentConePosition = 3;
            liftMotor.setTargetPosition(coneStack[currentConePosition]);
            liftMotor2.setTargetPosition(coneStack[currentConePosition]);
            liftMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            liftMotor2.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            liftMotor.setPower(-0.9);
            liftMotor2.setPower(-0.9);
            intakeServo.setPosition(0.25);
        }

        //manual lift override
//        if(backButton2 && !back2IsPressed){
//            back2IsPressed = true;
//            currentLiftPosition = 0;
//            LiftMotor.setPower(RightJoyY2);
//            if(RightJoyY2 == 0.0) {
//                LiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            }
//        } else if(!backButton2) {
//            back2IsPressed = false;
//        }

        holonomicDrive.teleopDrive(x/0.75,y/0.75,z/0.75);

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addLine("Current Lift position: " + currentLiftPosition + "\nCurrent Lift Ticks: " + liftPos[currentLiftPosition]);
        telemetry.addLine("Current Lift Motor Ticks: "  + liftMotor.getCurrentPosition());
        telemetry.addLine("intake open: "  + liftMotor.getCurrentPosition());
        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
