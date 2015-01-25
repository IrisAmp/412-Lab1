/*
 * January 21, 2015
 * Author Christoph Sydora, Brandon Yue
 *
 * Program takes command matrix that will control the power of each wheel and duration of movement.
 * during which time the program will calculate its estimated location and heading
 */

import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.NXTMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.robotics.EncoderMotor;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

public class Part3 {
	/**
	 * main function call
	 * 
	 * @param args
	 */
	public static void main(String[] args) {
		DeadReckoningRobot robot = new DeadReckoningRobot();

		int[][] command = { 
				{ 80, 60, 2 }, 
				{ 60, 60, 1 }, 
				{ -50, 80, 2 } 
				};

		robot.drive(command);
	}

	/**
	 * class for Dead Reckoning Robot
	 *
	 */
	public static class DeadReckoningRobot {
		private EncoderMotor motorL;
		private EncoderMotor motorR;
		private SampleProvider gyro;
		private float[] gyroSample;

		private double x = 0.0;
		private double y = 0.0;
		private double theta = 0.0;

		private int tachoR;
		private int tachoL;

		public static final double WHEEL_DIAMETER_CM = 5.7;
		public static final double WHEEL_R_CM = 12.0;

		public DeadReckoningRobot() {
			motorL = new NXTMotor(MotorPort.A);
			motorR = new NXTMotor(MotorPort.B);

			// Init the sensor
			Port p1 = LocalEV3.get().getPort("S3");
			@SuppressWarnings("resource")
			EV3GyroSensor sensor = new EV3GyroSensor(p1);
			gyro = sensor.getAngleMode();
			gyroSample = new float[gyro.sampleSize()];
		}

		/**
		 * drive function that drives route and calculates location and heading.
		 * 
		 * @param commands
		 */
		public void drive(int[][] commands) {

			// previous tachometer count of each wheel
			int tl0;
			int tr0;

			// wait before starting
			Button.waitForAnyPress();

			// initial tachometer count
			tl0 = motorL.getTachoCount();
			tr0 = motorR.getTachoCount();

			// initial angle count
			gyro.fetchSample(gyroSample, 0);
			float thetaA = gyroSample[0];

			// loop over each route type
			for (int[] command : commands) {

				// set the power for each wheel
				motorL.setPower(abs(command[0]));
				motorR.setPower(abs(command[1]));

				// set motion of each wheel
				if (command[0] < 0) {
					motorL.backward();
				} else {
					motorL.forward();
				}
				if (command[1] < 0) {
					motorR.backward();
				} else {
					motorR.forward();
				}

				// set up variables for calculating loaction and heading
				double distPerTick;
				double deltaDistance;
				double velocity;
				double ticksPerRot;
				double radPerTick;
				double deltaHeading;
				double rotVelocity;
				int dt = 10;

				// loop which will calculate change in location and heading over
				// 10ms intervals
				for (int j = 0; j < (command[2] * 1000); j += dt) {
					Delay.msDelay(dt);

					// take the difference between the new count and previous
					// count
					tachoL = motorL.getTachoCount() - tl0;
					tachoR = motorR.getTachoCount() - tr0;
					tl0 = motorL.getTachoCount();
					tr0 = motorR.getTachoCount();

					// calculate the changes in loaction
					distPerTick = ((WHEEL_DIAMETER_CM * Math.PI) / 360.0);
					deltaDistance = (((double) tachoL + (double) tachoR) / 2.0)
							* distPerTick;
					velocity = (deltaDistance / dt);
					ticksPerRot = (WHEEL_R_CM * Math.PI) / distPerTick;
					radPerTick = (2.0 * Math.PI) / ticksPerRot;
					deltaHeading = ((double) tachoR - (double) tachoL)
							* (radPerTick / 2.0);
					rotVelocity = deltaHeading / dt;

					// update the location and heading
					x += velocity * dt * Math.cos(theta);
					y += velocity * dt * Math.sin(theta);
					theta += rotVelocity * dt;
				}

				// stop the wheel motion
				motorL.stop();
				motorR.stop();
			}

			// final angle
			gyro.fetchSample(gyroSample, 0);
			float thetaB = gyroSample[0];

			// output the new location heading and wait for button press
			System.out.printf("x = %5.2f\n", x);
			System.out.printf("y = %5.2f\n", y);
			System.out.printf("w = %5.2f\n", (theta * 180 / Math.PI) % 360);
			System.out.printf("gyro i = %5.0f\n", thetaA);
			System.out.printf("gyro f = %5.0f\n", thetaB);

			Button.waitForAnyPress();
		}

		/**
		 * absolute value function
		 * 
		 * @param i
		 * @return absolute value of i
		 */
		private int abs(int i) {
			if (i < 0)
				return -i;
			else
				return i;
		}
	}
}