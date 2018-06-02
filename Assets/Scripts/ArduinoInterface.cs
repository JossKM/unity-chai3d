using System;
using UnityEngine;
using System.Collections;
using System.IO.Ports;
using System.IO;

public class ArduinoInterface : MonoBehaviour
{
    private SerialPort stream;
    public string portName;

    public static byte msg_CW = 255;
    public static byte msg_CCW = 1;

    public string lastMessage;

    void Start()
    {
        stream = new SerialPort(portName, 9600);
        stream.ReadTimeout = 20;
        try
        {
            stream.Open();
        }
        catch (IOException e)
        {
            Debug.Log("ERROR: No device found on port: " + portName);
            return;
        }
    }

    void Update()
    {
        if (stream.IsOpen)
        {
            if (Input.GetKeyUp(KeyCode.A))
            {
                //WriteString("A");
                WriteByte(msg_CW);
                Debug.Log("wrote: " + msg_CW);
            }

            if (Input.GetKeyUp(KeyCode.D))
            {
                //WriteString("D");
                WriteByte(msg_CCW);
                Debug.Log("wrote: " + msg_CCW);
            }
            lastMessage = ReadArduino();
        }
    }


    public virtual void WriteByte(byte value)
    {
        stream.Write(new byte[] { value }, 0, 1);
    }

    public void WriteString(string message)
    {
        stream.WriteLine(message);
        stream.BaseStream.Flush();
    }

    public string ReadArduino()
    {
        try
        {
            var message = stream.ReadLine();
            //Debug.Log("message received from Arduino: " + message);
        
        return message;
        }
        catch (TimeoutException e)
        {
            return null;
        }
    }
}
