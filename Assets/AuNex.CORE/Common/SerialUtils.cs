using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace AuNex
{
    namespace Common
    {
        public class SerialHandler
        {
            // シリアル通信のパラメータ
            private String portName;
            private int baudRate = 115200;
            private int readTimeout = 10;
            private int writeTimeout = 10;

            // シリアルポート
            private System.IO.Ports.SerialPort serialPort;

            // 受信をサブスレッドで実行する
            private System.Threading.Thread readThread;
            private bool isOpened = false;
            private System.Collections.Concurrent.ConcurrentQueue<string> receiveBuffer;

            public SerialHandler(String port_name, int baud_rate, int read_timeout, int write_timeout)
            {
                portName = port_name;
                baudRate = baud_rate;
                readTimeout = read_timeout;
                writeTimeout = write_timeout;

                isOpened = false;
            }

            public void OpenSerial()
            {
                try
                {
                    serialPort = new System.IO.Ports.SerialPort(portName, baudRate)
                    {
                        NewLine = "\n",
                        ReadTimeout = readTimeout,
                        WriteTimeout = writeTimeout
                    };

                    Debug.Log("Open Serial ...");
                    serialPort.Open();

                    new WaitForSeconds(1);
                    Debug.Log("Start read thread.");
                    isOpened = true;
                    readThread = new System.Threading.Thread(Read);
                    readThread.Start();
                }
                catch(Exception ex)
                {
                    Debug.LogError($"[Serial] Open failed: {ex.Message}");
                }
            }

            public String GetReadBuffer()
            {
                if(receiveBuffer.TryDequeue(out String data))
                {
                    return data;
                }
                else
                {
                    Debug.LogError("Failed to get data from buffer.");

                    return new String("ERROR");
                }
            }

            public void WriteSerial(String write_data)
            {
                if(serialPort == null)
                {
                    Debug.LogError("Port hasn't already opened .");
                    return;
                }

                serialPort.WriteLine(write_data);
            }

            public void CloseSerial()
            {
                isOpened = false;

                if(readThread != null && readThread.IsAlive)readThread.Join();

                if(serialPort != null && serialPort.IsOpen)serialPort.Close();
            }

            private void Read()
            {
                while(isOpened)
                {
                    try
                    {
                        string data = serialPort.ReadLine();
                        receiveBuffer.Enqueue(data);
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