using System;
using UnityEngine;
using System.Collections;
using System.IO.Ports;
using System.IO;

public class ArduinoInterface : MonoBehaviour
{
    private SerialPort stream;
    public string portName = "COM5";

    public static byte msg_CW = 255;
    public static byte msg_CCW = 1;

    public string lastMessage = "";

    public float minPos = 0.0f;
    public float maxPos = 6.0f;

    public string plungerMessageString = "sensor = ";
    public Transform syringePlunger;

    void Start()
    {
        stream = new SerialPort(portName, 9600);
        stream.ReadTimeout = 20;

        // connect to Arduino via Serial Port
        try
        {
            stream.Open();

            Debug.Log("Arduino connected on port: " + portName);
        }
        catch (IOException)
        {
            Debug.Log("ERROR: No device found on port: " + portName);
            return;
        }
    }

    void Update()
    {
        // if connected to arduino
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

            if (lastMessage != null)
            {
                if (lastMessage.StartsWith(plungerMessageString))
                {
                    string valueStr = lastMessage.Remove(0, plungerMessageString.Length);
                    int value = int.Parse(valueStr);

                    //Debug.Log(value);

                    Vector3 plungerPos = syringePlunger.localPosition;

                    float plungerDepth = Mathf.Lerp(minPos, maxPos, Mathf.InverseLerp(0, 1023, value));
                    plungerPos.y = plungerDepth;

                    syringePlunger.localPosition = plungerPos;
                }
            }
        }
        else
        {
            // attempt to connect again
            try
            {
                stream.Open();

                Debug.Log("Arduino connected on port: " + portName);
            }
            catch (IOException)
            {
            }
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
            string message;

            int numMessagesToReadPerFrame = 2;
            int i = 0;

            do
            {
                message = stream.ReadLine();
                i++;
            } while (i < numMessagesToReadPerFrame); // also leaves the loop when timeout occurs

            return message;
        }
        catch (TimeoutException e)
        {
            return null;
        }
    }
}
