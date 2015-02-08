package org.usfirst.frc.team1885.robot.comms;

import java.io.ObjectInputStream;
import java.net.Socket;

public class ClientTest {

	public static void main(String[] args) {
		try {
			Socket clientSocket = new Socket("team-1885.local", 4444);

			ObjectInputStream clientInputStream = new ObjectInputStream(
					clientSocket.getInputStream());

			Object msg;
			while (clientSocket.isConnected()) {
				msg = clientInputStream.readObject();

				System.out.println(msg);
			}
			
		} catch (Exception e) {
			e.printStackTrace();
		}

	}

}
