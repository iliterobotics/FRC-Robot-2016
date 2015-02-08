package org.usfirst.frc.team1885.robot.comms;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.ObjectInputStream;
import java.io.ObjectOutputStream;
import java.io.PrintWriter;
import java.net.ServerSocket;
import java.net.Socket;
import java.util.ArrayList;
import java.util.List;

import org.usfirst.frc.team1885.robot.common.type.ServerMessageType;

public class RobotServer implements Runnable {
	private static RobotServer instance = null;
	ServerSocket server;
	RobotClientConnection robo;
	String line;
	List<RobotServerListener> robotModules = new ArrayList<RobotServerListener>();
	ObjectInputStream inStream;
	ObjectOutputStream outStream;

	protected RobotServer() {
	}

	public static RobotServer getInstance() {
		if (instance == null)
			instance = new RobotServer();
		return instance;
	}

	private boolean isRunning;

	public boolean startServer() {
		if (!this.isRunning) {
			Thread serverThread = (new Thread(this));
			serverThread.start();
			// synchronized(serverThread) {
			// try {
			// serverThread.wait();
			// } catch (InterruptedException e) {
			// this.isRunning = false;
			// e.printStackTrace();
			// }
			// }
		}

		return this.isRunning;
	}

	public boolean stopServer() {
		if (this.isRunning) {
			this.isRunning = false;
		}

		return this.isRunning;
	}

	@Override
	public void run() {
		this.isRunning = true;
		// this.notify();
		while (this.isRunning) {
			try {
				// server.accept returns a client connection
				robo = new RobotClientConnection(server.accept());
				Thread t = new Thread(robo);
				t.start();
			} catch (IOException e) {
				System.out.println("Accept failed: 4444");
			}
		}

		robo.disconnect();

	}

	public void setup(int port) {
		try {
			server = new ServerSocket(port);
		} catch (IOException e) {
			System.out.println("Could not listen on port " + port);
		}
	}

	private class RobotClientConnection implements Runnable {
		private Socket client;
		private BufferedReader in;
		private PrintWriter out;

		public RobotClientConnection() {
		}

		public void run() {
			try {

				inStream = new ObjectInputStream(robo.client.getInputStream());
				outStream = new ObjectOutputStream(
						robo.client.getOutputStream());
				// out = new PrintWriter(client.getOutputStream(), true);
			} catch (IOException e) {
				System.out.println("in or out failed");
			}

			while (client.isConnected()) {
				try {
					receive();
					// RobotServerEvent roboEvent = new RobotServerEvent(msg);
					// // Is this the same message as the Send Method message?
					// notifyListeners(roboEvent);
				} catch (Exception e) {
					System.out.println("Read failed");
				}
			}
		}

		public void disconnect() {
			try {
				this.client.close();
			} catch (IOException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}

		RobotClientConnection(Socket socket) {
			client = socket;
		}
	}

	private class RobotClientDecryption {
		public void printLine() {
			System.out.println(line);
		}
	}

	public void addListener(RobotServerListener listener) {
		robotModules.add(listener);
	}

	public void notifyListeners(RobotServerEvent event) {
		for (RobotServerListener listener : robotModules) {
			listener.receivedServerEvent(event);
		}
	}

	public void receive() {
		try {
			Object dataObject = inStream.readObject();

			if (dataObject instanceof Message) {
				RobotServerEvent e = new RobotServerEvent(
						(Message) (dataObject));
				for (RobotServerListener r : robotModules) {
					notifyListeners(e);
				}
			}
		} catch (Exception e) {
			System.out.println("Error : " + e);
		}
	}

	public void send(Message message) {
		try {

			System.out.println("Telemetry Message Built");
			outStream.writeObject(message);
			outStream.flush();
			outStream.reset();
			System.out.println("Telemetry Message Sent");
		} catch (Exception e) {
			System.out.println("Error : " + e);
		}
	}
}

/*
 * 
 * Distinguish between Vision and Telemetry Messages :: if Vision, update
 * CameraDataService. if Telemetry, do something
 */

