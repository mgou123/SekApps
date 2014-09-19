package connectionProtocol;


import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;
import java.net.Socket;
import java.util.UUID;

import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothSocket;
import android.util.Log;

public class ConnParameters 
{
	private DataOutputStream dos;
	private DataInputStream dis;
	private Socket s;
	private boolean connEstablished;
	private boolean networkConn;
	/*UUID used to connect with Bluetooth server*/
	private final UUID uuid=UUID.fromString("00001101-0000-1000-8000-00805f9b34fb");
	private BluetoothSocket mmSocket;
	
	
	/*Constructor for establishing network connection*/
	public ConnParameters(String IP)
	{
		try {
			/*Connect to the robot's server*/
			s = new Socket(IP,12345);
			this.setS(s);
			this.setDos(new DataOutputStream(s.getOutputStream()));
			this.setDis(new DataInputStream(s.getInputStream()));
			connEstablished=true;
			networkConn=true;
		} catch (Exception e) {Log.i("ConnParameters","Could not connect to socket");
		connEstablished=false;}
	}
	
	/*Connection for establishing Bluetooth connection*/
	public ConnParameters(BluetoothDevice device)
	{
        /*Get a BluetoothSocket to connect with the given BluetoothDevice*/
        try {
        	this.setMmSocket(device.createRfcommSocketToServiceRecord(uuid));
        } catch (Exception e) {Log.i("ConnParameters","Cannot create socket"); }
		
		try {
            /* Connect the device through the socket. This will block
             * until it succeeds or throws an exception*/
            mmSocket.connect();
            this.setDos(new DataOutputStream(mmSocket.getOutputStream()));
			this.setDis(new DataInputStream(mmSocket.getInputStream()));
            connEstablished=true;
            networkConn=false;
        } catch (IOException connectException) {Log.i("ConnParameters","Cannot Connect to socket");
        	connEstablished=false;
        	/*Unable to connect; close the socket and get out*/
            try {
                mmSocket.close();
            } catch (Exception closeException) {Log.i("ConnParameters","constructor"); }
        }
	}
	
	public boolean isConnEstablished() {
		return connEstablished;
	}
	
	public Socket getS() {
		return s;
	}
	public void setS(Socket s) {
		this.s = s;
	}
	
	public DataOutputStream getDos() {
		return dos;
	}
	
	public void setDos(DataOutputStream dos) {
		this.dos = dos;
	}
	
	public  DataInputStream getDis() {
		return dis;
	}
	public void setDis(DataInputStream dis) {
		this.dis = dis;
	}
	
	public BluetoothSocket getMmSocket() {
		return mmSocket;
	}
	public void setMmSocket(BluetoothSocket mmSocket) {
		this.mmSocket = mmSocket;
	}
	
	public boolean isNetworkConn() {
		return networkConn;
	}
	public void setNetworkConn(boolean networkConn) {
		this.networkConn = networkConn;
	}

	public void closeConnection()
	{
		try {
			if(networkConn)
				s.close();
			else
				mmSocket.close();
		} catch (Exception e) {Log.i("ConnectionParam", "closeConnection");}
		s=null;
		mmSocket=null;
		dos=null;
		dis=null;
	}

}
