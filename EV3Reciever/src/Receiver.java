/*
 * September 21, 2009
 * Author by Tawat Atigarbodee
 *
 * Install this program on to NXT brick and use it with NXTremoteControl_TA.java
 *
 * To use this program.
 *  -   Install Lejos 0.8.5
 *  -   Include Lejos_nxj library to the project path
 *  -   Upload the program using lejosdl.bat (I use Eclipse)
 *  -   To exit the program, restart NXT brick (remove battery)
 * 
 * NXT setup
 *  -  Port A for right wheel
 *  -  Port C for left wheel
 *  -  No sensor is needed
 *  
 * Note: This program is a partial of my project file. 
 * I use “USBSend” and “USBReceive” created by Lawrie Griffiths 
 * as a pattern for creating USB communication between PC and NXT. 
 */

import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;

import lejos.hardware.motor.NXTMotor;
import lejos.hardware.port.MotorPort;
import lejos.remote.nxt.BTConnection;
import lejos.remote.nxt.BTConnector;
import lejos.remote.nxt.NXTConnection;
import lejos.robotics.EncoderMotor;

public class Receiver {
	public static DataOutputStream dataOut;
	public static DataInputStream dataIn;
	public static BTConnection BTLink;
	public static BTConnection btLink;
	public static int speed = 50, turnSpeed = 50, speedBuffer, speedControl;
	public static int commandValue, transmitReceived;
	public static boolean[] control = new boolean[6];
	public static boolean[] command = new boolean[6];

	private static EncoderMotor motorL;
	private static EncoderMotor motorR;

	public static void main(String[] args) {
		motorL = new NXTMotor(MotorPort.A);
		motorR = new NXTMotor(MotorPort.B);

		connect();

		while (true) {
			control = checkCommand();
			speedControl = getSpeed(control);
			move(control, speedControl);
		}
	}// End main

	public static boolean[] checkCommand()// check input data
	{

		try {
			transmitReceived = dataIn.readInt();

			if (transmitReceived == 1) {
				command[0] = true;
			}// forward
			if (transmitReceived == 10) {
				command[0] = false;
			}
			if (transmitReceived == 2) {
				command[1] = true;
			}// backward
			if (transmitReceived == 20) {
				command[1] = false;
			}
			if (transmitReceived == 3) {
				command[2] = true;
			}// leftTurn
			if (transmitReceived == 30) {
				command[2] = false;
			}
			if (transmitReceived == 4) {
				command[3] = true;
			}// rightTurn
			if (transmitReceived == 40) {
				command[3] = false;
			}
			if (transmitReceived == 5) {
				command[0] = false;// stop
				command[1] = false;
				command[2] = false;
				command[3] = false;
			}
			if (transmitReceived == 6) {
				command[4] = true;
			}// speed up
			if (transmitReceived == 60) {
				command[4] = false;
			}
			if (transmitReceived == 7) {
				command[5] = true;
			}// slow down
			if (transmitReceived == 70) {
				command[5] = false;
			} else {
			}
			;
		}

		catch (IOException ioe) {
			System.out.println("IO Exception readInt");
		}
		return command;

	}// End checkCommand

	public static void move(boolean[] D, int S) {
		int movingSpeed;
		boolean[] direction = new boolean[4];

		direction[0] = D[0];
		direction[1] = D[1];
		direction[2] = D[2];
		direction[3] = D[3];

		movingSpeed = S;

		motorL.setPower(movingSpeed);
		motorR.setPower(movingSpeed);

		if (direction[0] == true) {
			motorL.forward();
			motorR.forward();
		}

		if (direction[1] == true) {
			motorL.backward();
			motorR.backward();
		}

		if (direction[2] == true) {
			motorL.setPower(turnSpeed);
			motorR.setPower(turnSpeed);
			motorL.forward();
			motorR.backward();
		}

		if (direction[3] == true) {
			motorL.setPower(turnSpeed);
			motorR.setPower(turnSpeed);
			motorL.backward();
			motorR.forward();
		}

		if (direction[0] == true && direction[2] == true) {
			speedBuffer = (int) (movingSpeed * 1.5);

			motorL.setPower(speedBuffer);
			motorR.forward();
			motorL.forward();
		}

		if (direction[0] == true && direction[3] == true) {
			speedBuffer = (int) (movingSpeed * 1.5);

			motorR.setPower(speedBuffer);
			motorR.forward();
			motorL.forward();
		}

		if (direction[1] == true && direction[2] == true) {
			speedBuffer = (int) (movingSpeed * 1.5);

			motorL.setPower(speedBuffer);
			motorR.backward();
			motorL.backward();
		}

		if (direction[1] == true && direction[3] == true) {
			speedBuffer = (int) (movingSpeed * 1.5);

			motorR.setPower(speedBuffer);
			motorR.backward();
			motorL.backward();
		}

		if (direction[0] == false && direction[1] == false
				&& direction[2] == false && direction[3] == false) {
			motorL.stop();
			motorR.stop();
		}

	}// End move

	public static void connect() {
	    System.out.println("Listening");
	    BTConnector conn = new BTConnector();
	    NXTConnection conn2 = conn.waitForConnection(-1, BTConnection.PACKET);
	    dataOut = conn2.openDataOutputStream();
	    dataIn = conn2.openDataInputStream();
	}// End connect

	public static int getSpeed(boolean[] D) {
		boolean accelerate, decelerate;

		accelerate = D[4];
		decelerate = D[5];

		if (accelerate == true) {
			speed += 50;
			command[4] = false;
		}

		if (decelerate == true) {
			speed -= 50;
			command[5] = false;
		}

		return speed;
	}// End getSpeed

}// NXTtr Class
