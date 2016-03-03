package org.usfirst.frc.team2823.robot;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;

public class TalkToPi {
	
	private static final TalkToPi singleton;
	static {
		TalkToPi value = null;
		try {
			value = new TalkToPi();
		} catch (IOException e) {
			System.err.println("Unable to find address of PI");
		}
		singleton = value;
	}

	private final InetAddress pi;
	private static DatagramSocket serverSocket;

	private TalkToPi() throws IOException {
		pi = InetAddress.getByName("raspberrypi.local");
		serverSocket = new DatagramSocket(9876);
	}

	public static void rawCommand(String command) {
		if (singleton != null) {
			try {
				singleton.udp(command);
			} catch (IOException e) {
				System.err.println("Error sending message to pi");
				e.printStackTrace();
			}
		} else {
			System.err.println("WARNING: No raspberry pi connection");
		}
	}

	private void udp(String command) throws IOException {
		byte[] sendData = command.getBytes();
		DatagramPacket sendPacket = new DatagramPacket(sendData,
				sendData.length, pi, 5201);
		serverSocket.send(sendPacket);
	}

	public static String recv() throws IOException {
        byte[] receiveData = new byte[1024];
        DatagramPacket receivePacket = new DatagramPacket(receiveData, receiveData.length);
        serverSocket.receive(receivePacket);
        return new String(receivePacket.getData());
    }

    public static void close() {
        serverSocket.close();
    }
}
