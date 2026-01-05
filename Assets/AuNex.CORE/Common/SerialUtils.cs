using System;
using System.Collections;
using System.Collections.Generic;
using System.IO.Ports;
using UnityEngine;

namespace AuNex
{
    namespace Common
    {
        public class SerialHandler
        {
            public enum ReadType
            {
                STRING,
                BYTE
            }

            public struct ReadData
            {
                public string str_data;
                public byte[] byte_data;
            }

            // シリアル通信のパラメータ
            private String portName;
            private int baudRate = 115200;
            private int readTimeout = 10;
            private int writeTimeout = 10;
            private ReadType readType = ReadType.STRING;

            // シリアルポート
            private System.IO.Ports.SerialPort serialPort;

            // 受信をサブスレッドで実行する
            private System.Threading.Thread readThread;
            private bool isOpened = false;
            private bool newMessageReceived = false;
            private string receiveBuffer;
            private List<byte> receiveBufferByte;

            public SerialHandler(String port_name, int baud_rate, int read_timeout=10, int write_timeout=10)
            {
                portName = port_name;
                baudRate = baud_rate;
                readTimeout = read_timeout;
                writeTimeout = write_timeout;

                isOpened = false;
                receiveBuffer = new String("error");
                receiveBufferByte = new();
            }

            public void SetReadType(ReadType type)
            {
                readType = type;
            }

            public void OpenSerial()
            {
                try
                {
                    serialPort = new System.IO.Ports.SerialPort(portName, baudRate, System.IO.Ports.Parity.None, 8, System.IO.Ports.StopBits.One)
                    {
                        NewLine = "\n",
                        ReadTimeout = readTimeout,
                        WriteTimeout = writeTimeout
                    };

                    Debug.Log("Open Serial ...");
                    serialPort.Open();
                    isOpened = true;
                    
                    Debug.Log("Start read thread.");
                    readThread = new System.Threading.Thread(Read);
                    readThread.Start();
                }
                catch(Exception ex)
                {
                    Debug.LogError($"[Serial] Open failed: {ex.Message}");
                }
            }

            public ReadData ReadSerial()
            {
                if(newMessageReceived)
                {
                    var read_data = new ReadData();
                    read_data.byte_data = receiveBufferByte.ToArray();
                    read_data.str_data = receiveBuffer;

                    receiveBufferByte.Clear();
                    newMessageReceived = false;

                    return read_data;
                }
                else
                {
                    var read_data = new ReadData();
                    read_data.byte_data = new byte[1];
                    read_data.str_data = "ERROR";
                    return read_data;
                }
            }

            public void WriteSerial(String write_data)
            {
                if(serialPort == null)
                {
                    Debug.LogError("Port hasn't already opened .");
                    return;
                }

                try
                {
                    serialPort.WriteLine(write_data);
                }
                catch(TimeoutException)
                {
                    
                }
            }

            public void WriteSerial(byte[] data, int size)
            {
                if(serialPort == null)
                {
                    Debug.LogError("Port hasn't already opened .");
                    return;
                }

                try
                {
                    serialPort.Write(data, 0, size);
                }
                catch(TimeoutException)
                {
                    
                }
            }
            public void CloseSerial()
            {
                isOpened = false;

                if(readThread != null && readThread.IsAlive)readThread.Join();

                if(serialPort != null && serialPort.IsOpen)serialPort.Close();
            }

            private void Read()
            {
                while(isOpened && serialPort != null && serialPort.IsOpen)
                {
                    try
                    {
                        if(readType == ReadType.STRING)
                        {
                            receiveBuffer = serialPort.ReadLine();
                            newMessageReceived = true;
                        }
                        else if(readType == ReadType.BYTE)
                        {
                            byte[] buf = new byte[1];
                            int read = serialPort.Read(buf, 0, 1);

                            if(read == 0)continue;

                            if(buf[0] == 0x0A)
                            {
                                newMessageReceived = true;
                            }

                            receiveBufferByte.Add(buf[0]);
                        }
                    }
                    catch(TimeoutException)
                    {
                        // Timeout時はなにもしない
                    }
                    catch(Exception ex)
                    {
                        Debug.LogError($"[Serial] Open failed: {ex.Message}");
                    }
                }
            }
        }
    }
}