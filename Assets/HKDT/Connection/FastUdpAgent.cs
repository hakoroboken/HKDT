using UnityEngine;
using System.Net;
using System.Net.Sockets;
using System.Threading;
using System.Collections.Concurrent;
using ROS2;


namespace HKDT.Connection
{
    public class FastUdpAgent
    {
        [Header("UDP通信設定")]
        private string AgentIP = "192.168.11.5";
        private int AgentPort = 64201;

        // UDPの変数
        private UdpClient udpClient;
        private IPEndPoint myEndPoint;

        // 受信Thread
        private Thread receiveThread;
        
        // ROS2変数
        private IPublisher<std_msgs.msg.UInt8MultiArray> receiveDataPublisher;
        private ISubscription<std_msgs.msg.UInt8MultiArray> sendDataSubscriber;
        private IPublisher<std_msgs.msg.Bool> statusPublisher;

        ConcurrentQueue<byte[]> sendData;
        public FastUdpAgent(string ip, int port)
        {
            AgentIP = ip;
            AgentPort = port;

            myEndPoint = new(IPAddress.Parse(AgentIP), AgentPort);

        }

        public bool Start(ROS2Node node, string send_topic_name, string receive_topic_name, string status_topic_name)
        {
            receiveDataPublisher = node.CreatePublisher<std_msgs.msg.UInt8MultiArray>(receive_topic_name);
            sendDataSubscriber = node.CreateSubscription<std_msgs.msg.UInt8MultiArray>(send_topic_name, SendDataCallback);
            statusPublisher = node.CreatePublisher<std_msgs.msg.Bool>(status_topic_name);

            try
            {
                udpClient = new UdpClient(myEndPoint);
                udpClient.Client.ReceiveTimeout = 3000;

                sendData = new();

                receiveThread = new Thread(UdpCallback);
                receiveThread.Start();

                return true;
            }
            catch(SocketException)
            {
                return false;
            }
        }

        private void SendDataCallback(std_msgs.msg.UInt8MultiArray msg)
        {
            sendData.Enqueue(msg.Data);
        }

        private void UdpCallback()
        {
            while(true)
            {
                try
                {
                    IPEndPoint senderIP = null;
                    byte[] receiveBuffer = udpClient.Receive(ref senderIP);

                    // サイズ１はping pong用
                    if(receiveBuffer.Length != 1)
                    {
                        var receiveMsg = new std_msgs.msg.UInt8MultiArray
                        {
                            Data = receiveBuffer
                        };

                        receiveDataPublisher.Publish(receiveMsg);
                    }

                    if(sendData.TryDequeue(out var result))
                    {
                        udpClient.SendAsync(result, result.Length, senderIP);
                    }
                    else
                    {
                        byte[] pingData = new byte[1]{0};
                        udpClient.SendAsync(pingData, pingData.Length, senderIP);
                    }

                    var status_msg = new std_msgs.msg.Bool
                    {
                        Data = true
                    };
                    statusPublisher.Publish(status_msg);
                }
                catch (SocketException)
                {
                    // Debug.LogWarning($"受信エラー：{e}");
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