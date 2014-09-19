package connectionProtocol;


import java.io.File;
import java.io.FileOutputStream;
import java.util.ArrayList;
import android.bluetooth.BluetoothDevice;
import android.os.Environment;
import android.util.Log;

public class Connection 
{
	/*Matrix that holds canonical values in range [-1,1] of current position using as reference 
	point the one set by the user*/
	private float[] canonicalAz;
	/*Different state requires different communication protocol*/
	private int state;
	/*True if the robot is moving*/
	private boolean power;
	/*User is/not controlling robot but may resume*/
	private boolean onPause;
	/* True if the next touch event is a location */
	private boolean Dest,Start;
	/* True if the robot is on automatic navigation */
	private boolean navigating;
	/*Floor map */
	private File map=null;
	/*Variable that holds the connection parameters*/
	private ConnParameters connP;
	/* meters in every pixel of the map */
	private final float resolution=0.05f;
	/* the last position that Sec was */
	private Quaternion lastPos;
	/* indicates whether there is an active http stream or not */
	private boolean streaming;
	/* List containing destination points */
	private ArrayList<Quaternion> points;
	
	/** @param returns Arraylist Points containing all destination points*/
	public ArrayList<Quaternion> getPoints() {
		return points;
	}
	/** @param returns true if device is connected to an Http stream */
	public boolean isStreaming() {
		return streaming;
	}
	public void setStreaming(boolean streaming) {
		this.streaming = streaming;
	}
	/** Initiate all variables **/
	public Connection()
	{
		this.canonicalAz=new float[3];
		onPause=power=navigating=streaming=false;
		points=new ArrayList<Quaternion>();
	}
	/** Connect to IP via wireless connection
	 * @return returns false if connection failed
	 * @param IP string that contains IP address to connect*/
	public boolean connect(String IP)
	{
		this.connP=new ConnParameters(IP);
		return connP.isConnEstablished();
	}
	/** Connect with Bluetooth
	 * @return returns false if connection failed
	 * @param device is the Bluetooth device to connect*/
	public boolean connect(BluetoothDevice device)
	{
		this.connP=new ConnParameters(device);
		return connP.isConnEstablished();
	}
	/** @return true if start point is about to be given */
	public boolean isStart() {
		return Start;
	}
	public void setStart(boolean start) {
		Start = start;
	}
	/** @return true if destination point is about to be given */
	public boolean isDest() {
		return Dest;
	}
	public void setDest(boolean dest) {
		Dest = dest;
	}
	/** @return true if robot is on automatic navigation*/
	public boolean isNavigating() {
		return navigating;
	}
	public void setNavigating(boolean navigating) {
		this.navigating = navigating;
	}
	/** Cancels automatic navigation */
	public void cancelRoute()
	{
		Log.i("Canceled Route", "");
		this.setState(6);
	}
	
	/** Create the folder "/Skel" in devices SD card, where the map will be stored */
	private boolean createFolder()
	{
		File folder = new File(Environment.getExternalStorageDirectory() + "/Skel");
		if (!folder.exists()) {
		    return folder.mkdir();
		}
		return false;
	}
	
	/** Set the position of the robot.  
	 * If it is a destination,it is added to Arraylist Points
	 * to be sent when user decides to initiate navigation*/
	public void setPoint(Quaternion q,boolean start)
	{
		if(!start)/* Point is destination */
			this.getPoints().add(q);
		else/* Point is robots position */
		{
			this.setState(4);
		
			try{
				connP.getDis().read(new byte[4]);
				/* x coordinate of starting point */
				connP.getDos().write(Float.toString(q.getX()*resolution).getBytes());
				connP.getDos().flush();
				connP.getDis().read(new byte[4]);
				/* y coordinate of starting point */
				connP.getDos().write(Float.toString(q.getY()*resolution).getBytes());
				connP.getDos().flush();
				connP.getDis().read(new byte[4]);
				/* x and y coordinates of quaternion rotaion are always zero
				 * since robot can not rotate relative axis x or y
				 * z coordinate of quaternion rotation */
				connP.getDos().write(Float.toString(q.getZ()).getBytes());
				connP.getDos().flush();
				connP.getDis().read(new byte[4]);
				/* w coordinate of quaternion rotation */
				connP.getDos().write(Float.toString(q.getW()).getBytes());
				connP.getDos().flush();
			 }catch(Exception ex){Log.i("Connection", "setPoint");}
		}
	}
	
	/** Save the current map */
	public void saveMap()
	{
		try{
			/* send 11 ,operation code to save the map */
			connP.getDis().read(new byte[4]);
			connP.getDos().write(Float.toString(14.0f).getBytes());
			connP.getDos().flush();
			connP.getDis().read(new byte[4]);
			/*Send normalized value for right and left movement [-1,1]*/
			connP.getDos().write(Float.toString(14.0f).getBytes());
			connP.getDos().flush();
		 }catch(Exception ex){Log.i("Connection", "saveMap");}
	}
	
	/** Sends all destination points to the robot in order to initiate navigation */
	public void sendPoints()
	{
		/* If list contains more than one point a vector is sent instead of
		 * a single point */
		try{
			if(this.points.size()==1)
			{	this.setState(5);Log.i("1 point", " 1");}
			else
			{	
				Log.i("polla point"," points = "+this.getPoints().size());
				this.setState(13);
				connP.getDis().read(new byte[4]);
				/* Send the size of the list */
				/*connP.getDos().write(Integer.toString(this.getPoints().size()).getBytes());
				connP.getDos().flush();*/
				this.send(this.getPoints().size());
			}
			for(Quaternion q:this.getPoints())
			{
				Log.i("For", "loop");
				connP.getDis().read(new byte[4]);
				/* x coordinate of starting point */
				connP.getDos().write(Float.toString(q.getX()*resolution).getBytes());
				connP.getDos().flush();
				connP.getDis().read(new byte[4]);
				/* y coordinate of starting point */
				connP.getDos().write(Float.toString(q.getY()*resolution).getBytes());
				connP.getDos().flush();
				connP.getDis().read(new byte[4]);
				/* x and y coordinates of quaternion rotation are always zero
				 * since robot can not rotate relative axis x or y
				 * z coordinate of quaternion rotation */
				connP.getDos().write(Float.toString(q.getZ()).getBytes());
				connP.getDos().flush();
				connP.getDis().read(new byte[4]);
				/* w coordinate of quaternion rotation */
				connP.getDos().write(Float.toString(q.getW()).getBytes());
				connP.getDos().flush();
				Log.i("For", "end loop");
			}
			Log.i("sendpoints", "prin to read");
			connP.getDis().read(new byte[4]);
			Log.i("sendpoints", "meta to read");
		}catch(Exception ex){Log.i("Connection", "sendPoints");}	
	}
	
	/** @return robots position in coordinates relative to the map */
	public Quaternion getPosition()
	{
		lastPos=new Quaternion();
		/* buffer to hold file size */
		byte point[]=new byte[18];
		
		try {
			connP.getDis().read(new byte[4]);
			connP.getDos().write(Integer.toString(1).getBytes());
			connP.getDos().flush();

			connP.getDis().read(point);
			lastPos.setY(Float.parseFloat(new String(point,"utf-8")));
			
			connP.getDos().write(Integer.toString(1).getBytes());
			connP.getDos().flush();
			
			point=new byte[18];
			connP.getDis().read(point);
			
			lastPos.setX(Float.parseFloat(new String(point,"utf-8")));
			
		} catch (NumberFormatException e) {
			Log.i("Exception in getPosition","Not a float");
		} catch (Exception e) {e.printStackTrace();
			Log.i("Exception in getPosition","socket");
		}
		return lastPos;
	}
	
	/** 
	 * Receive Map. 
	 * Downloads the map (in PNG format)
	 * to the device
	 * @return true if map was successfully received*/
	public boolean receiveMap()
	{
		/*Send to the robot the operation code 3 which is the code for 
		 * receiving the map*/
		this.setState(3);
		/* Buffer to store images data */
		byte buffer[]=new byte[1048];
		/* buffer to hold file size */
		byte Stringsize[]=new byte[5];
		File f;
		
		try {
				connP.getDos().write(Integer.toString(1).getBytes());
				connP.getDos().flush();
				/*Create folder to hold the map */
				if(createFolder())
					Log.i("Created new folder","/Skel");
				f = new File(Environment.getExternalStorageDirectory()+"/Skel"+File.separator+"FloorMap.png");
				
				if(f.exists())
					f.delete();
				f.createNewFile();
				
				connP.getDis().read(Stringsize);
				int size=Integer.parseInt(new String(Stringsize,"utf-8"));
				Log.i("Size "+size," ");
				connP.getDos().write(Integer.toString(1).getBytes());
				connP.getDos().flush();
				/*write the bytes in file*/
				FileOutputStream fo = new FileOutputStream(f);
				int len=0;
				long received=0;
				while(received<size)
				{
					len=connP.getDis().read(buffer);
					fo.write(buffer,0,len);
					received+=len;
				}
				Log.i("Received "+received," ");
				fo.flush();
				/*close the FileOutput */
				fo.close();
				Log.i("File received","File path : "+f.getAbsolutePath());
				connP.getDos().write(Integer.toString(1).getBytes());
				connP.getDos().flush();
			} catch (Exception e) {
				Log.d("Connection","receiveMap");
				e.printStackTrace();return false;}
			map=f;
		return true;
	}
	/** @return absolute path of the map in devices storage space */
	public String getFloorMap()
	{
		return  "/mnt/sdcard/Skel/FloorMap.png";
	}
	
	/** sends the integer move to the robot */
	public void send(int move)
	{
		try {
			connP.getDos().write(Integer.toString(move).getBytes());
			connP.getDos().flush();
		} catch (Exception e) {Log.i("Connection", "send(int)");}
	}
	/** sends the float move to the robot */
	public void send(float move)
	{
		try {
			connP.getDos().write(Float.toString(0.0f).getBytes());
			connP.getDos().flush();
		} catch (Exception e) {Log.i("Connection", "send(float)");}
	}
	
	/** Initiates camera streaming at 
	 * http://x.x.x.x:8081/stream?topic=/camera/rgb/image_color.
	 * Variable controlling indicates whether user is on control
	 * mode or not */
	public void stream(boolean controlling)
	{	
		try{
			send(8);
			connP.getDis().read(new byte[4]);
			setStreaming(true);
		 }catch(Exception ex){Log.i("Connection", "Stream");}
	}
	/** method used when user is on control mode*/
	public void beginManeuver()
	{
		try{
			connP.getDis().read(new byte[4]);
			send(9);
		 }catch(Exception ex){Log.i("Connection", "beginManeuver");}
	}
	
	public void beginManeuverMap()
	{
		try{/* server is waiting to read state */
			send(9);
			connP.getDis().read(new byte[4]);
		 }catch(Exception ex){Log.i("Connection", "beginManeuverMap");}
	}
	public void endManeuverMap()
	{
		try{
			send(10);
			connP.getDis().read(new byte[4]);
		 }catch(Exception ex){Log.i("Connection", "endManeuverMap");}
	}
	public void endManeuver()
	{
		try{
			connP.getDis().read(new byte[4]);
			send(10);
		 }catch(Exception ex){Log.i("Connection", "endManeuver");}
	}
	public void beginRecording()
	{
		try{
			connP.getDis().read(new byte[4]);
			send(11);
		 }catch(Exception ex){Log.i("Connection", "beginRecording");}
	}
	public void endRecording()
	{
		try{
			connP.getDis().read(new byte[4]);
			send(12);
		 }catch(Exception ex){Log.i("Connection", "endRecording");}
	}
	public void setSensitivity(float m,float t)
	{
		try {
			connP.getDos().write(Float.toString(m).getBytes());
			connP.getDos().flush();
			connP.getDis().read(new byte[4]);
			connP.getDos().write(Float.toString(t).getBytes());
			connP.getDos().flush();
		} catch (Exception e) {Log.i("Connection", "send(float)");}
	}
	
	public void send(float[] az)
	{
		this.setCanonicalAz(az);
		
		try{
		    if(power)
			{
				connP.getDis().read(new byte[4]);
				//Using division with 30.0 to produce range from [-1,1]
				//Send normalized value for front and backwards movement [-1,1]
				connP.getDos().write(Float.toString(canonicalAz[2]).getBytes());
				connP.getDos().flush();
				connP.getDis().read(new byte[4]);
				//Send normalized value for right and left movement [-1,1]
				connP.getDos().write(Float.toString(canonicalAz[1]).getBytes());
				connP.getDos().flush();
			}
		 }catch(Exception ex){Log.i("Connection", "send");}
	}
	
	/** every time user exits control mode, pause() must be called before onDestroy*/
	public void pause()
	{
		try {
			//Send value 13 to exit sensor control
			connP.getDis().read(new byte[4]);
			connP.getDos().write(Float.toString(13.0f).getBytes());
			connP.getDos().flush();
			connP.getDis().read(new byte[4]);
			connP.getDos().write(Float.toString(13.0f).getBytes());
			connP.getDos().flush();
		} catch (Exception e) {Log.i("Connection","pause");}
	}
	/** stops robots movement */
	public void stop()
	{
		try {
			//Sending 0.0 , 0.0 to stop moving
	    	connP.getDis().read(new byte[4]);
			connP.getDos().write(Float.toString(0.0f).getBytes());
			connP.getDos().flush();
			connP.getDis().read(new byte[4]);
			connP.getDos().write(Float.toString(0.0f).getBytes());
			connP.getDos().flush();
		} catch (Exception e) {	Log.i("Connection", "stop");}
		
	}
	/** honks the horn*/
	public void horn()
	{
		try {
			connP.getDis().read(new byte[4]);
			//Using division with 30.0 to produce range from [-1,1]
			//Send normalized value for front and backwards movement [-1,1]
			connP.getDos().write(Float.toString(15.0f).getBytes());
			connP.getDos().flush();
			connP.getDis().read(new byte[4]);
			//Send normalized value for right and left movement [-1,1]
			connP.getDos().write(Float.toString(15.0f).getBytes());
			connP.getDos().flush();
		} catch (Exception e) {Log.i("Connection", "horn");}

	}
	/** called when application is finished*/
	public void exit()
	{
		try {
			connP.getDos().write(Integer.toString(0).getBytes());
		} catch (Exception e) {Log.i("Connection", "exit");}
		connP.closeConnection();
	}
	
	public boolean isOnPause() {
		return onPause;
	}

	public void setOnPause(boolean onPause) {
		this.onPause = onPause;
	}
	
	public int getState() {
		return state;
	}

	public void setState(int state) {
		this.state = state;
		Log.i("State = "," "+state);
		try {
			connP.getDos().write(Integer.toString(state).getBytes());
			connP.getDos().flush();
		} catch (Exception e) {Log.i("Connection", "setState");}
	}
	
	public boolean isPower() {
		return power;
	}
	
	public void setCanonicalAz(float[] canonicalAz) {
		System.arraycopy(canonicalAz, 0, this.canonicalAz, 0, 3);
	}

	public void setPower(boolean power) {
		this.power = power;
	}
	
	public void exitMap()
	{
		setNavigating(false);
		setPower(false);
	}
}
