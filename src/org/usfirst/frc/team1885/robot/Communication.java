package org.usfirst.frc.team1885.robot;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.PrintWriter;
import java.net.ServerSocket;
import java.net.Socket;

public class Communication {
	ServerSocket server;
	RobotClientConnection robo;
	String line;

	public void listenSocket() {
		try {
			server = new ServerSocket(4444);
		} catch (IOException e) {
			System.out.println("Could not listen on port 4444");
			System.exit(-1);
		}
		while (true) {
			try {
				// server.accept returns a client connection
				robo = new RobotClientConnection(server.accept());
				Thread t = new Thread(robo);
				t.start();
			} catch (IOException e) {
				System.out.println("Accept failed: 4444");
				System.exit(-1);
			}
		}
	}

	private class RobotClientConnection implements Runnable {
		Socket client;
		BufferedReader in;
		PrintWriter out;

		public void run() {
			try {
				in = new BufferedReader(new InputStreamReader(
						client.getInputStream()));
				out = new PrintWriter(client.getOutputStream(), true);
			} catch (IOException e) {
				System.out.println("in or out failed");
				System.exit(-1);
			}

			while (true) {
				try {
					line = in.readLine();

				} catch (IOException e) {
					System.out.println("Read failed");
					System.exit(-1);
				}
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
}