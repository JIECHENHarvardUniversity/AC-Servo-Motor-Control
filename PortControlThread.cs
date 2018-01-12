using UnityEngine;
using System.Collections;
using System.IO.Ports;
using System;
using System.Collections.Generic;
using System.Threading;
using UnityEngine.UI;

public class PortControlThread : MonoBehaviour {

	//public GUIText gui;
	public string ReceiveMsg="";
	public string ErrorMsg="";
	public string SendMsg="";
    public string SendMsg2 = "";
    public string portName = "COM1";
	public int baudRate = 115200;
	private Parity parity = Parity.None;
	private int dataBits = 8;
	private StopBits stopBits = StopBits.One;
	
	int[] data = new int[32];//用于存储6位数据
	byte[] dataSend = new byte[32];//用于存储6位数据
	private SerialPort sp = null;
	Thread dataReceiveThread;
	private string sValue2="0";

	public GUIStyle style1;
	public GUIStyle style2;

    public GameObject obj;
    public string[]  arrPorts;
    public Slider SenderSpeedControl;
   // public Text SenderSpeedShow;
	void Start()
	{
		dataReceiveThread = new Thread(new ThreadStart(DataReceiveFunction));

		arrPorts=System.IO.Ports.SerialPort.GetPortNames();

     

    }

	void StartPortOpen()
	{
		if(sp!=null && sp.IsOpen)
		{
			ClosePort();
		}
		OpenPort();
		if(dataReceiveThread!=null && dataReceiveThread.IsAlive)
		{
			dataReceiveThread.Abort();
		}
		dataReceiveThread.Start();

	}

    void SetStart(int index,int s)
    {
     
        if (index==s)
        {
            SendMsg += "{";
        }
        if (index == s + 4)
        {
            SendMsg += "}";
        }
    }


   
	void Update()
	{
       obj. transform.localRotation = Quaternion.Euler(gPitch, 0, gRoll);
        Vector3 ps = obj.transform.position;
        obj.transform.position = new Vector3(ps.x, gz*0.02f+0.89f, ps.z);

        ReceiveMsg = "";
		for(int i=0; i<data.Length; i++)
		{
			ReceiveMsg += data[i].ToString() + " ";
		}


		SendMsg="";
		for(int i=0; i<dataSend.Length; i++)
		{
         
            //SetStart(i, 14);
           // SetStart(i,18);
           // SetStart(i, 22);
            SendMsg += Convert.ToString( dataSend[i],16).ToUpper()+ " ";//.ToString(16) + " ";
		}
	
	}
	
	public void OpenPort()
	{
		sp = new SerialPort(portName, baudRate, parity, dataBits, stopBits);
		sp.ReadTimeout = 400;
		try
		{
			sp.Open();
		}
		catch(Exception ex)
		{
			ErrorMsg=ex.Message;
			Debug.Log(ex.Message);
		}
	}
	
	public void ClosePort()
	{
		try
		{
			if(sp!=null)
				sp.Close();
			if(dataReceiveThread!=null && dataReceiveThread.IsAlive)
			{
				dataReceiveThread.Abort();
			}
		}
		catch(Exception ex)
		{
			Debug.Log(ex.Message);
		}
	}
	public void Empty()//前后
	{
		dataSend[0]=0xFB;
		dataSend[1]=0xFD;
		for(int i=2;i<31;i++)
		{
			dataSend[i]=0x0;
		}
		
		int iSpeed=0;
		try{
			iSpeed=	int.Parse(sValue2);
		}catch
		{
			
		}
		
		iSpeed=iSpeed%128;
		if(iSpeed>0)
		{
			dataSend[26]=(byte) iSpeed;
		}


		int crc=0;
		for(int i=1;i<31;i++)
		{
			crc+=dataSend[i];
		}
		crc=crc%128;
		dataSend[31]=(byte) crc;
		
		if(sp!=null && sp.IsOpen)
		{
			sp.Write(dataSend,0,dataSend.Length);
			
		}
		 
	}

    float gz = 0.0f;

	

    public void MyUpdate(float roll_angle,float pitch_angle,float z)
    {
        dataSend[0] = 0xFB;
        dataSend[1] = 0xFD;

        for (int i = 2; i < 10; i++)
        {
            dataSend[i] = 0x0;
        }


        byte[] b_z = BitConverter.GetBytes(z);
        for (int i = 0; i < 4; i++)
        {
            dataSend[10 + i] = b_z[i];
        }


        byte[] b_roll = BitConverter.GetBytes(roll_angle);

        for (int i = 0; i < 4; i++)
        {
            dataSend[14 + i] = b_roll[i];
        }


        byte[] b_pitch = BitConverter.GetBytes(pitch_angle);
        for (int i = 0; i < 4; i++)
        {
            dataSend[18 + i] = b_pitch[i];
        }


      

        for (int i = 22; i < 31; i++)
        {
            dataSend[i] = 0x0;
        }



        int iSpeed = 0;
        try
        {
            iSpeed = int.Parse(sValue2);
        }
        catch
        {

        }

        iSpeed = 0;// iSpeed % 128;
        if (iSpeed > 0)
        {
            dataSend[26] = (byte)iSpeed;
        }


        int crc = 0;
        for (int i = 1; i < 31; i++)
        {
            crc += dataSend[i];
        }
        crc = crc % 256;
        dataSend[31] = (byte)crc;

        if (sp != null && sp.IsOpen)
        {
            sp.Write(dataSend, 0, dataSend.Length);

        }
    }



    public   byte[] ToBytes(decimal value)
	{

		int[] bits = decimal.GetBits(value);
		//byte[] bytes = new byte[bits.Length * 4]; 
//		for (int i = 0; i < bits.Length; i++)
//		{
//			for (int j = 0; j < 4; j++)
//			{
//				bytes[i * 4 + j] = (byte)(bits[i] >> (j * 8));
//			}
//		}

		byte[] bytes =new byte[4];
		for (int i = 0; i < bits.Length; i++)
		{
//			for (int j = 0; j < 4; j++)
//			{
//				bytes[i * 4 + j] = (byte)(bits[i] >> (j * 8));
//			}
			bytes[i] = (byte)(bits[i] >> (i * 8));
		}
		return bytes;
	}
	/// <summary>
	/// 将字节数组转换为<c>Decimal</c>对象。
	/// </summary>
	/// <param name="array">要转换的字节数组。</param>
	/// <returns>所转换的<c>Decimal</c>对象。</returns>
	public   decimal FromBytes(byte[] array)
	{
		int[] bits = new int[array.Length / 4];
		for (int i = 0; i < bits.Length; i++)
		{
			for (int j = 0; j < 4; j++)
			{
				bits[i] |= array[i * 4 + j] << j * 8;
			}
		}
		return new decimal(bits);
	}
    public void WriteData(string dataStr)
	{
		if(sp!=null && sp.IsOpen)
		{
			sp.Write(dataStr);

			//sp.Write(byte[],offset,count);
		}
		
	}
	void OnApplicationQuit()
	{
		ClosePort ();
	}	
	void DataReceiveFunction()
	{
		byte[] buffer = new byte[128];
		int bytes = 0;
		int flag0 = 0xFF;
		int flag1 = 0xAA;
		int index = 0;//用于记录此时的数据次序
		while (true)
		{
			if (sp != null && sp.IsOpen)
			{
				try
				{
					bytes = sp.Read(buffer, 0, buffer.Length);
					for(int i=0; i<bytes; i++)
					{
						
						if(buffer[i]==flag0 || buffer[i]==flag1)
						{
							index = 0;//次序归0 
							continue;
						}
						else
						{
							if(index>=data.Length)	index = data.Length-1;//理论上不应该会进入此判断，但是由于传输的误码，导致数据的丢失，使得标志位与数据个数出错
							data[index] = buffer[i];//将数据存入data中
							index++;
						}
						
					}
				}
				catch (Exception ex)
				{
					if (ex.GetType() != typeof(ThreadAbortException))
					{
						Debug.Log(ex.Message);
					}
				}
			}
			Thread.Sleep(10);
		}
	}



    public GUISkin curskin;
    int time = 0;
    int gRoll = 0;
    int  gPitch = 0;

 int SendSpeedValue=0;

    int GetControlSpeed()
    {
        float v = SenderSpeedControl.value;
        if (v == 0)
            v = 0.5f;
        int s = (int)(33 * (1 / v));
        return s;
    }
	void OnGUI()
	{

        //连接端口
        GUI.skin = curskin;
        GUI.color = Color.black;
		GUI.Label(new Rect(10,10,140,40),"端口",style1);
		portName=GUI.TextField(new Rect(160,10,160,30),portName);

		string sPorts="[";
		for(int i=0;i<this.arrPorts.Length;i++)
		{
			sPorts+=arrPorts[i]+",";

		}
		sPorts+="]";
		GUI.Label(new Rect(400,10,200,40),sPorts,style1);

		if(GUI.Button(new Rect(50,60,100,40),"打开端口"))
		{
			OpenPort();
		}

		if(GUI.Button(new Rect(160,60,100,40),"关闭端口"))
		{
			ClosePort();
		}



        GUI.Label(new Rect(340, 210, 100, 40), "速度值(0-10)", style1);
        sValue2 = GUI.TextField(new Rect(480, 210, 160, 30), sValue2);
     //   GUI.Label(new Rect(10, 210, 140, 40), "移动数量");
        //sValue = GUI.TextField(new Rect(160, 210, 160, 30), sValue);
        //float v = float.Parse(sValue);
        time++;

        if (GUI.Button(new Rect(50, 250, 50, 40), "左"))
        {
            gRoll++;
        }
      if (GUI.Button(new Rect(100, 250, 50, 40), "右"))
        {
            gRoll--;
        }


        if (GUI.Button(new Rect(200, 250, 80, 40), "前抬"))
        {
            gPitch--;
        }
            if (GUI.Button(new Rect(280, 250, 80, 40), "后升"))
               {
                   gPitch++;
               }


              if (GUI.Button(new Rect(400, 250, 80, 40), "垂直升"))
                {
                    gz++;
                }
                if (GUI.Button(new Rect(480, 250, 80, 40), "垂直降"))
                {
                    gz--;
                }

          if ((time % GetControlSpeed() )== 0)
          MyUpdate(gRoll,gPitch, gz);




        if (GUI.Button(new Rect(580,250,100,40),"回正"))
		{
            gRoll = 0;
            gPitch = 0;
            gz = 0;
	
		}


		GUI.Label(new Rect(10,310,600,40),SendMsg);
        GUI.Label(new Rect(10, 350, 600, 40),"Roll:"+gRoll+"Pitch:"+gPitch+"z:"+gz);

       // SenderSpeedShow.text = "value:" + SenderSpeedControl.value;






    }

}