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

public class RobotServer implements Runnable {
	private static RobotServer instance = null;
	ServerSocket server;
	RobotClientConnection robo;
	String line;
	List<RobotServerListener> robotModules = new ArrayList<RobotServerListener>();
	ObjectInputStream inStream;
	ObjectOutputStream outStream;

	List<Message> messages = new ArrayList<Message>();
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
			try {
				serverThread.wait();
			} catch (InterruptedException e) {
				this.isRunning = false;
				e.printStackTrace();
			}
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
		this.notify();
		while (this.isRunning) {
			try {
				//server.accept returns a client connection
				robo = new RobotClientConnection(server.accept());
				Thread t = new Thread(robo);
				t.start();
			} catch (IOException e) {
				System.out.println("Accept failed: 4444");
				System.exit(-1);
			}
		}

		robo.disconnect();

	}

	public void setup(int port) {
		try {
			server = new ServerSocket(port);
		} catch (IOException e) {
			System.out.println("Could not listen on port " + port);
			System.exit(-1);
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
				in = new BufferedReader(new InputStreamReader(
						client.getInputStream()));
				out = new PrintWriter(client.getOutputStream(), true);
			} catch (IOException e) {
				System.out.println("in or out failed");
				System.exit(-1);
			}

			while (client.isConnected()) {
				try {
					line = in.readLine();
					System.out.println("read: " + line);
//					RobotServerEvent roboEvent = new RobotServerEvent(msg); // Is this the same message as the Send Method message?
//					notifyListeners(roboEvent);
				} catch (IOException e) {
					System.out.println("Read failed");
					System.exit(-1);
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
			inStream = new ObjectInputStream(robo.client.getInputStream());
			TelemmetryMessage teleMessage = (TelemmetryMessage) (inStream
					.readObject());
			System.out.println("Telemmetry Message Built.");
			messages.add(teleMessage);
			System.out.println("Telemmetry Message Stored.");
		} catch (Exception e) {
			System.out.println("Error : " + e);
		}
	}

	public void send(Message message) {
		try {
			outStream = new ObjectOutputStream(robo.client.getOutputStream());

			System.out.println("Telemmetry Message Built");
			outStream.writeObject(message);
			System.out.println("Telemmetry Message Sent");
		} catch (Exception e) {
			System.out.println("Error : " + e);
		}
	}
}
