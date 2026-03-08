using UnityEngine;
using System.Net;
using System.Net.Sockets;
using System.Threading;
using System.Collections.Concurrent;
using ROS2;


namespace HKDT.Connection
{
    public class FastUdpServer
    {
        [Header("UDP通信設定")]
        private string ServerIP = "192.168.11.1";
        private int ServerPort = 64201;
        private string AgentIP = "192.168.11.1";
        private int AgentPort = 64202;

        // UDPの変数
        private UdpClient udpClient;
        private IPEndPoint serverEndPoint;
        private IPEndPoint agentEndPoint;
        // 受信Thread
        private Thread receiveThread;
        
        private IPublisher<std_msgs.msg.UInt8MultiArray> receiveDataPublisher;
        private ISubscription<std_msgs.msg.UInt8MultiArray> sendDataSubscriber;
        private IPublisher<std_msgs.msg.Bool> statusPublisher;

        ConcurrentQueue<byte[]> sendData;
        // Start is called before the first frame update
        public FastUdpServer(string server_ip, int server_port, string agent_ip, int agent_port)
        {
            ServerIP = server_ip;
            ServerPort = server_port;
            AgentIP = agent_ip;
            AgentPort = agent_port;

            serverEndPoint = new(IPAddress.Parse(ServerIP), ServerPort);
            agentEndPoint = new(IPAddress.Parse(AgentIP), AgentPort);
        }

        // Update is called once per frame
        public bool Start(ROS2Node node, string send_topic_name, string receive_topic_name, string status_topic_name)
        {
            receiveDataPublisher = node.CreatePublisher<std_msgs.msg.UInt8MultiArray>(receive_topic_name);
            sendDataSubscriber = node.CreateSubscription<std_msgs.msg.UInt8MultiArray>(send_topic_name, SendDataCallback);
            statusPublisher = node.CreatePublisher<std_msgs.msg.Bool>(status_topic_name);

            try
            {
                udpClient = new UdpClient(serverEndPoint);
                udpClient.Connect(agentEndPoint);
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
                    if(sendData.TryDequeue(out var result))
                    {
                        udpClient.SendAsync(result, result.Length, agentEndPoint);
                    }
                    else
                    {
                        byte[] pingData = new byte[1]{0};
                        udpClient.SendAsync(pingData, pingData.Length);
                    }

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

                    var status_msg = new std_msgs.msg.Bool
                    {
                        Data = true
                    };
                    statusPublisher.Publish(status_msg);
                }
                catch (SocketException)
                {
                    // Debug.LogWarning($"[{NodeName}] 受信エラー：{e}");
                    var status_msg = new std_msgs.msg.Bool
                    {
                        Data = true
                    };
                    statusPublisher.Publish(status_msg);
                }
            }
        }
    }
}// end of namespace