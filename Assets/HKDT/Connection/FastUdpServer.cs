using UnityEngine;
using System.Net;
using System.Net.Sockets;
using System.Threading;
using System.Collections.Concurrent;
using ROS2;

public class FastUdpServer : MonoBehaviour
{
    [Header("UDP通信設定")]
    public string ServerIP = "192.168.11.1";
    public int ServerPort = 64201;
    public string AgentIP = "192.168.11.1";
    public int AgentPort = 64202;

    [Header("ROS2設定")]
    public string NodeName = "FastUdpServer";
    public string SendTopicName = "/udp/send";
    public string ReceiveTopicName = "/udp/receive";

    // UDPの変数
    private UdpClient udpClient;
    private IPEndPoint serverEndPoint;
    private IPEndPoint agentEndPoint;

    // 受信Thread
    private Thread receiveThread;
    
    // ROS2変数
    private ROS2UnityComponent ros2unity;
    private ROS2Node node;
    private IPublisher<std_msgs.msg.ByteMultiArray> receiveDataPublisher;
    private ISubscription<std_msgs.msg.ByteMultiArray> sendDataSubscriber;

    ConcurrentQueue<byte[]> sendData;
    // Start is called before the first frame update
    void Start()
    {
        serverEndPoint = new(IPAddress.Parse(ServerIP), ServerPort);
        agentEndPoint = new(IPAddress.Parse(AgentIP), AgentPort);

        ros2unity = GetComponent<ROS2UnityComponent>();
    }

    // Update is called once per frame
    void Update()
    {
        if(ros2unity.Ok())
        {
            if(node == null)
            {
                Debug.Log($"[{NodeName}]ROS2を初期化します");
                InitializeNode();

                Debug.Log($"[{NodeName}] UDPを初期化します");
                Debug.Log($"[{NodeName}] サーバーIP:{serverEndPoint} \nエージェントIP:{agentEndPoint}");
                InitializeUDP();
            }
        }
    }

    private void InitializeNode()
    {
        node = ros2unity.CreateNode(NodeName);

        receiveDataPublisher = node.CreatePublisher<std_msgs.msg.ByteMultiArray>(ReceiveTopicName);

        sendDataSubscriber = node.CreateSubscription<std_msgs.msg.ByteMultiArray>(
            SendTopicName,
            SendDataCallback
        );
    }

    private void InitializeUDP()
    {
        udpClient = new UdpClient(serverEndPoint);
        udpClient.Connect(agentEndPoint);
        udpClient.Client.ReceiveTimeout = 3000;

        sendData = new();

        receiveThread = new Thread(UdpCallback);
        receiveThread.Start();
    }

    private void SendDataCallback(std_msgs.msg.ByteMultiArray msg)
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
                    udpClient.Send(pingData, pingData.Length);
                }

                IPEndPoint senderIP = null;
                byte[] receiveBuffer = udpClient.Receive(ref senderIP);

                // サイズ１はping pong用
                if(receiveBuffer.Length != 1)
                {
                    var receiveMsg = new std_msgs.msg.ByteMultiArray
                    {
                        Data = receiveBuffer
                    };

                    receiveDataPublisher.Publish(receiveMsg);
                }
            }
            catch (SocketException e)
            {
                Debug.LogWarning($"[{NodeName}] 受信エラー：{e}");
            }
        }
    }
}
