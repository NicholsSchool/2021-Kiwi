package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class KiwiMap
{
    public BNO055IMU imu;
    public DcMotor frontMotor, rightMotor, leftMotor;
    public RevBlinkinLedDriver blinkin;
    
    public boolean anchorless = false;

    HardwareMap hwMap;

    public KiwiMap() {}

    public void init( HardwareMap ahwMap ) 
    {
        hwMap = ahwMap;
        
        
        /*
         *  IMU
         */
        
        // Instantiating IMU Parameters, setting angleUnit...
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        
        // Defining and Initializing IMU... Initializing it with the above Parameters...
        imu = hwMap.get( BNO055IMU.class, "imu" );
        imu.initialize( parameters );
        
        
        /*
         *  BLINKIN
         */

        // Defining and Initializing Blinkin...
        blinkin = hwMap.get( RevBlinkinLedDriver.class, "Blinkin" );


        /*
         *  MOTORS
         */
         
        // Defining and Initializing the Motors...
        frontMotor = hwMap.get( DcMotor.class, "Motor1" );
        rightMotor = hwMap.get( DcMotor.class, "Motor2" );
        leftMotor = hwMap.get( DcMotor.class, "Motor3" );
        
        // Inverting the Motors...
        frontMotor.setDirection( DcMotorSimple.Direction.REVERSE );
        rightMotor.setDirection( DcMotorSimple.Direction.REVERSE );
        leftMotor.setDirection( DcMotorSimple.Direction.REVERSE );
        
        // Setting to run using Encoders...
        frontMotor.setMode( DcMotor.RunMode.RUN_USING_ENCODERS );
        rightMotor.setMode( DcMotor.RunMode.RUN_USING_ENCODERS );
        leftMotor.setMode( DcMotor.RunMode.RUN_USING_ENCODERS );
        
        // Setting Brake as 0 Power Behavior...
        frontMotor.setZeroPowerBehavior( DcMotor.ZeroPowerBehavior.BRAKE );
        rightMotor.setZeroPowerBehavior( DcMotor.ZeroPowerBehavior.BRAKE );
        leftMotor.setZeroPowerBehavior( DcMotor.ZeroPowerBehavior.BRAKE );
    }
    
    
    /*
     *  IMU
     */
    
    /**
     * Gets the robot's current Heading, ( the robot's current angular orientation 
     *  on an elevated-floor-like axis ). 
     * 
     * @return the robot's current Heading
     */
    public float getHeading() 
    {   
        return imu.getAngularOrientation().firstAngle;
    }

    
    /**
     * Moves the robot. No spinning, no saucing. Just linear-like, across-the-floor
     *  movement.
     * 
     * @param speed the speed the robot will move at
     * @param angle the angle at which the robot will move
     */
    public void move( double speed, double angle ) 
    {    
        frontMotor.setPower( speed * Math.cos( ( Math.PI / 180 ) * ( angle ) ) );
        rightMotor.setPower( speed * Math.cos( ( Math.PI / 180 ) * ( angle + 120 ) ) );
        leftMotor.setPower( speed * Math.cos( ( Math.PI / 180 ) * ( angle + 240 ) ) );
    }
    
    /**
     * Spins the robot. No linear movement, so no saucin'. Just spinning in place.
     * 
     * @param speed the speed the robot will spin at
     */
    public void spin( double speed ) 
    {
        if( anchorless ) {
            frontMotor.setPower( speed );
            rightMotor.setPower( speed );
            leftMotor.setPower( speed );
        }
        else {
            rightMotor.setPower( speed );
            leftMotor.setPower( speed );
        }
    }
    
    /**
     * Combines spinning and linear-like movement to sauce. The robot will move across 
     *  the floor while spinning. (Moves on a line, spinning all the while...)
     * 
     * @param speed the speed the robot will move across the floor (on a line) at
     * @param spinSpeed the speed the robot will spin at
     * @param angle the angle at which the robot will move across the floor (on a line) at
     * @param deltaHeading how far, in degrees, the robot has spun on the elevated-floor-like 
     *  axis. The change in Heading. Is updated with every loop
     */
    public void sauce( double speed, double spinSpeed, double angle, float deltaHeading ) 
    {    
        if( anchorless ) {
            frontMotor.setPower( spinSpeed + ( speed * Math.cos( ( Math.PI / 180 ) * ( angle + ( 0 - deltaHeading ) ) ) ) );
            rightMotor.setPower( spinSpeed + ( speed * Math.cos( ( Math.PI / 180 ) * ( angle + ( 120 - deltaHeading ) ) ) ) );
            leftMotor.setPower( spinSpeed + ( speed * Math.cos( ( Math.PI / 180 ) * ( angle + ( 240 - deltaHeading ) ) ) ) );
        } 
        else {
            frontMotor.setPower( speed * Math.cos( ( Math.PI / 180 ) * ( angle + ( 0 - deltaHeading ) ) ) );
            rightMotor.setPower( spinSpeed + ( speed * Math.cos( ( Math.PI / 180 ) * ( angle + ( 120 - deltaHeading ) ) ) ) );
            leftMotor.setPower( spinSpeed + ( speed * Math.cos( ( Math.PI / 180 ) * ( angle + ( 240 - deltaHeading ) ) ) ) );
        } 
    }
    
    
    /*
     *  MISC.
     */
    
    /**
     * Sets the 'front' of the robot to one of 3 'fronts.' Each 'front' is the same 
     *  color as a button on the gamepad, and, depending on what button is pressed,
     *  the respective 'front' is set to be the 'front.'
     * 
     * @param button the button pressed
     */
    public void setFront( char button ) 
    {    
        switch( button ) 
        {    
            case 'x':
                frontMotor = hwMap.get( DcMotor.class, "Motor3" );
                rightMotor = hwMap.get( DcMotor.class, "Motor1" );
                leftMotor = hwMap.get( DcMotor.class, "Motor2" );
                break;
            case 'y':
                frontMotor = hwMap.get( DcMotor.class, "Motor1" );
                rightMotor = hwMap.get( DcMotor.class, "Motor2" );
                leftMotor = hwMap.get( DcMotor.class, "Motor3" );
                break;
            case 'b':
                frontMotor = hwMap.get( DcMotor.class, "Motor2" );
                rightMotor = hwMap.get( DcMotor.class, "Motor3" );
                leftMotor = hwMap.get( DcMotor.class, "Motor1" );
        }
    }
    
    /**
     * Stops the robot. More accurately, stops all 3 of the robot's motors. 
     */
    public void stop() 
    {    
        frontMotor.setPower( 0 );
        rightMotor.setPower( 0 );
        leftMotor.setPower( 0 );
    }
}
