using System;
using UnityEngine;


// マルチスレッド化
using System.Threading;
// シリアル通信
using System.IO.Ports;

// ROS2
using ROS2;
using System.Collections.Concurrent;

public class FastSerial : MonoBehaviour
{
    // シリアル通信のパラメータ
    [Header("シリアル通信設定")]
    public string portName="/dev/ttyACM0";
    public int baudRate = 115200;
    public int readTimeout = 10;
    public int writeTimeout = 10;

    // ROS2パラメータ
    [Header("ROS2設定")]
    public string nodeName = "FastSerial";
    public string writeTopicName = "/serial/write";
    public string readTopicName = "/serial/read";

    // シリアルポート
    private SerialPort serialPort;
    private bool isOpened = false;
    // 受信を行うスレッド
    private Thread readThread;

    // ROS2
    private ROS2UnityComponent ros2unity;
    private ROS2Node node;
    // シリアル通信で受信したデータをROS2ネットワークに送信する
    private IPublisher<std_msgs.msg.ByteMultiArray> readDataPublisher;
    // シリアル通信で送信するデータをROS2ネットワークから受信する
    private ISubscription<std_msgs.msg.ByteMultiArray> writeDataSubscriber;

    ConcurrentQueue<byte[]> writeData;

    public void Start()
    {
        ros2unity = GetComponent<ROS2UnityComponent>();
    }

    public void Update()
    {
        if(ros2unity.Ok())
        {
            if(node == null)
            {
                Debug.Log($"[{nodeName}]ノードを初期化します");
                initializeNode();
            }
            else
            {
                Debug.Log($"[{nodeName}]シリアル通信を開始します");
                OpenSerial();
            }
        }
        else
        {
            Debug.LogError($"[{nodeName}]ROS2が正常でない");
        }
    }

    public void Oestroy()
    {
        CloseSerial();
        Debug.Log($"[{nodeName}]シリアル通信を終了しました");
        return;
    } 

    /// <summary>
    /// ROS2のパブリッシャーとサブスクライバを初期化する
    /// </summary>
    private void initializeNode()
    {
        node = ros2unity.CreateNode(nodeName);

        readDataPublisher = node.CreatePublisher<std_msgs.msg.ByteMultiArray>(readTopicName);

        writeDataSubscriber = node.CreateSubscription<std_msgs.msg.ByteMultiArray>(
            writeTopicName,
            writeDataCallback
        );
    }

    /// <summary>
    /// ROS2サブスクライバがデータを受信した際に実行する関数
    /// </summary>
    /// <param name="msg"></param>
    private void writeDataCallback(std_msgs.msg.ByteMultiArray msg)
    {
        writeData.Enqueue(msg.Data);
    }

    /// <summary>
    /// シリアル通信を開始する
    /// </summary>
    private void OpenSerial()
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
            isOpened = true;
            
            readThread = new Thread(SerialCallback);
            readThread.Start();
        }
        catch(Exception ex)
        {
            Debug.LogError($"[{nodeName}] シリアル通信を開始できませんでした。: {ex.Message}");
        }
    }

    /// <summary>
    /// シリアル通信を終了する
    /// </summary>
    public void CloseSerial()
    {
        isOpened = false;

        if(readThread != null && readThread.IsAlive)readThread.Join();

        if(serialPort != null && serialPort.IsOpen)serialPort.Close();
    }

    /// <summary>
    /// シリアル通信で受信した際に実行する関数
    /// </summary>
    private void SerialCallback()
    {
        while(isOpened && serialPort != null && serialPort.IsOpen)
        {
            try
            {
                byte[] buffer = new byte[256];
                int readSize = serialPort.Read(buffer, 0, 256);

                if(readSize > 0)
                {
                    byte[] read_data = new byte[readSize];
                    Array.Copy(buffer, read_data, readSize);

                    var read_data_msg = new std_msgs.msg.ByteMultiArray();

                    readDataPublisher.Publish(read_data_msg);

                    if(writeData.TryDequeue(out var write_buffer))
                    {
                        serialPort.Write(write_buffer, 0, write_buffer.Length);
                    }
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