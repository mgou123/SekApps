package com.example.roboskel;

import android.bluetooth.BluetoothDevice;
import connectionProtocol.Connection;

public class ActiveConnection 
{
	private static Connection c;
	private static boolean connected;

	public static boolean isConnected() {
		return connected;
	}

	public ActiveConnection()
	{
		c=new Connection();
	}
	
	public boolean connect(BluetoothDevice device)
	{
		connected=c.connect(device);
		return connected;
	}
	
	public boolean connect(String IP)
	{
		connected=c.connect(IP);
		return connected;
	}
	/** Returns Active connection */
	public static Connection getConn()
	{
		return c;
	}
	
	public static void CloseConnection()
	{
		try{
				c.exit();
				c=null;
			}catch(Exception e){}
	}
}
