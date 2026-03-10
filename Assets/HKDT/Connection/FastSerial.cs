using System;
using UnityEngine;
using System.Collections.Concurrent;

// マルチスレッド化
using System.Threading;
// シリアル通信
using System.IO.Ports;

// ROS2
using ROS2;

namespace HKDT.Connection
{
    public class FastSerial
    {
        // シリアル通信のパラメータ
        [Header("シリアル通信設定")]
        private string portName="/dev/ttyACM0";
        private int baudRate = 115200;
        private int readTimeout = 10;
        private int writeTimeout = 10;

        private bool onlyWrite;
        private bool onlyRead;

        // シリアルポート
        private SerialPort serialPort;
        // 受信を行うスレッド
        private Thread readThread;

        // ROS2
        // シリアル通信で受信したデータをROS2ネットワークに送信する
        private IPublisher<std_msgs.msg.UInt8MultiArray> readDataPublisher;
        private IPublisher<std_msgs.msg.Bool> statusPublisher;
        // シリアル通信で送信するデータをROS2ネットワークから受信する
        private ISubscription<std_msgs.msg.UInt8MultiArray> writeDataSubscriber;

        ConcurrentQueue<byte[]> writeData;

        public FastSerial(string port_name, int baud_rate, int read_timeout=10, int write_timeout=10, bool only_write=false, bool only_read=false)
        {
            portName = port_name;
            baudRate = baud_rate;
            readTimeout = read_timeout;
            writeTimeout = write_timeout;

            onlyRead = only_read;
            onlyWrite = only_write;
        }

        /// <summary>
        /// PublisherとSubscriberを初期化しシリアル通信を開始します
        /// </summary>
        /// <param name="node">ノード</param>
        /// <param name="write_topic_name">シリアル送信するデータ</param>
        /// <param name="read_topic_name">シリアル受信したデータ</param>
        /// /// <param name="status_topic_name">シリアル通信の状態</param>
        /// <returns>初期化成功したらtrue、失敗ならfalse</returns>
        public bool Start(ROS2Node node, string write_topic_name, string read_topic_name, string status_topic_name)
        {
            readDataPublisher = node.CreatePublisher<std_msgs.msg.UInt8MultiArray>(read_topic_name);
            writeDataSubscriber = node.CreateSubscription<std_msgs.msg.UInt8MultiArray>(write_topic_name, WriteDataCallback);
            statusPublisher = node.CreatePublisher<std_msgs.msg.Bool>(status_topic_name);

            return OpenSerial();
        }

        private void WriteDataCallback(std_msgs.msg.UInt8MultiArray msg)
        {
            writeData.Enqueue(msg.Data);
        }

        private bool OpenSerial()
        {
            try
            {
                serialPort = new SerialPort(portName, baudRate, Parity.None, 8, StopBits.One)
                {
                    NewLine = "\n",
                    ReadTimeout = readTimeout,
                    WriteTimeout = writeTimeout
                };

                serialPort.Open();
                
                if(onlyRead)
                {
                    readThread = new Thread(OnlyReadCallback);
                }
                else if(onlyWrite)
                {
                    readThread = new Thread(OnlyWriteCallback);
                }
                else
                {
                    readThread = new Thread(SerialCallback);
                }
                readThread.Start();

                writeData = new();

                return true;
            }
            catch(Exception ex)
            {
                Debug.LogError($"シリアル通信を開始できませんでした。: {ex.Message}");

                return false;
            }
        }

        /// <summary>
        /// シリアル通信を終了する
        /// </summary>
        public void CloseSerial()
        {
            if(readThread != null && readThread.IsAlive)readThread.Join();

            if(serialPort != null && serialPort.IsOpen)serialPort.Close();
        }

        private void SerialCallback()
        {
            while(true)
            {
                try
                {
                    byte[] buffer = new byte[256];
                    int readSize = serialPort.Read(buffer, 0, 256);

                    if(readSize > 0)
                    {
                        byte[] read_data = new byte[readSize];
                        Array.Copy(buffer, read_data, readSize);

                        var read_data_msg = new std_msgs.msg.UInt8MultiArray
                        {
                            Data = read_data
                        };

                        readDataPublisher.Publish(read_data_msg);

                        if(writeData.TryDequeue(out var write_buffer))
                        {
                            serialPort.Write(write_buffer, 0, write_buffer.Length);
                        }
                        else
                        {
                            byte[] pingData = new byte[1]{0};
                            serialPort.Write(pingData, 0, 1);
                        }

                        var status_msg = new std_msgs.msg.Bool
                        {
                            Data = true
                        };
                        statusPublisher.Publish(status_msg);
                    }
                }
                catch(TimeoutException)
                {
                    // Timeout時はなにもしない
                    // Debug.Log($"[{nodeName}] たいむあうと〜");
                    var status_msg = new std_msgs.msg.Bool
                    {
                        Data = false
                    };
                    statusPublisher.Publish(status_msg);
                }
                catch (Exception)
                {
                    var status_msg = new std_msgs.msg.Bool
                    {
                        Data = false
                    };
                    statusPublisher.Publish(status_msg);
                }
            }
        }

        private void OnlyReadCallback()
        {
            while(true)
            {
                try
                {
                    byte[] buffer = new byte[256];
                    int readSize = serialPort.Read(buffer, 0, 256);

                    if(readSize > 0)
                    {
                        byte[] read_data = new byte[readSize];
                        Array.Copy(buffer, read_data, readSize);

                        var read_data_msg = new std_msgs.msg.UInt8MultiArray
                        {
                            Data = read_data
                        };

                        readDataPublisher.Publish(read_data_msg);

                        var status_msg = new std_msgs.msg.Bool
                        {
                            Data = true
                        };
                        statusPublisher.Publish(status_msg);
                    }
                }
                catch(TimeoutException)
                {
                    // Timeout時はなにもしない
                    // Debug.Log($"[{nodeName}] たいむあうと〜");
                    var status_msg = new std_msgs.msg.Bool
                    {
                        Data = false
                    };
                    statusPublisher.Publish(status_msg);
                }
                catch (Exception)
                {
                    var status_msg = new std_msgs.msg.Bool
                    {
                        Data = false
                    };
                    statusPublisher.Publish(status_msg);
                }
            }
        }

        private void OnlyWriteCallback()
        {
            while(true)
            {
                try
                {
                    if(writeData.TryDequeue(out var write_buffer))
                    {
                        serialPort.Write(write_buffer, 0, write_buffer.Length);
                    }
                    else
                    {
                        byte[] pingData = new byte[1]{0};
                        serialPort.Write(pingData, 0, 1);
                    }

                    var status_msg = new std_msgs.msg.Bool
                    {
                        Data = true
                    };
                    statusPublisher.Publish(status_msg);
                }
                catch(TimeoutException)
                {
                    // Timeout時はなにもしない
                    // Debug.Log($"[{nodeName}] たいむあうと〜");
                    var status_msg = new std_msgs.msg.Bool
                    {
                        Data = false
                    };
                    statusPublisher.Publish(status_msg);
                }
                catch (Exception)
                {
                    var status_msg = new std_msgs.msg.Bool
                    {
                        Data = false
                    };
                    statusPublisher.Publish(status_msg);
                }
            }
        }
    }
}// end of namespace