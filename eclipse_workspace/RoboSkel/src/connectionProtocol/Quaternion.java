package connectionProtocol;

public class Quaternion 
{

	private float x,y,z,w;
	public float getX() {
		return x;
	}
	/** set axis x coordinates */
	public void setX(float x) {
		this.x = x;
	}
	public float getY() {
		return y;
	}
	/** set axis y coordinates */
	public void setY(float y) {
		this.y = y;
	}
	public float getZ() {
		return z;
	}
	/** set orientation of the robot */
	public void setZ(float z) {
		this.z = z;
	}
	public float getW() {
		return w;
	}
	public void setW(float w) {
		this.w = w;
	}
	public void copy(Quaternion q)
	{
		this.x=q.getX();
		this.y=q.getY();
		this.z=q.getZ();
		this.w=q.getW();
	}
	public Quaternion(float x,float y,float z,float w)
	{
		this.x=x;
		this.y=y;
		this.z=z;
		this.w=w;
	}
	/** Quaternion : (x,y,z) (x1,y1,z1,w1) 
	 * object contains only (x,y,z1,w1) all other values are zero
	 */
	public Quaternion()
	{
		this.x=this.y=this.w=0.0f;
		this.z=1.0f;
	}
}
