using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using ROS2;
using System;

public class M5StackBridge : MonoBehaviour
{
    // [Header("シリアル通信設定")]
    // public String m5_serial_port = "/dev/ttyACM0";
    // public int baud_rate = 115200;
    // private AuNex.Common.SerialHandler serialHandler;
    // [Header("ROS2設定")]
    // public String node_name = "m5stack_bridge";
    // public String color_topic_name = "/m5stack/color";
    // public String imu_topic_name = "/m5stack/imu";
    // public String btn_topic_name = "/m5stack/btn";
    // public String sensor_frame_id = "m5stack";
    // private ROS2UnityComponent ros2Unity;
    // private ROS2Node node;
    // private ISubscription<std_msgs.msg.ColorRGBA> color_subscriber;
    // private IPublisher<sensor_msgs.msg.Imu> imu_publisher;
    // private IPublisher<std_msgs.msg.Int16MultiArray> btn_publisher;
    // private std_msgs.msg.ColorRGBA color;

    // Start is called before the first frame update
    void Start()
    {
        // ros2Unity = GetComponent<ROS2UnityComponent>();

        // // 各変数を代入してSerialHandlerを初期化
        // serialHandler = new AuNex.Common.SerialHandler(
        //     m5_serial_port,
        //     baud_rate,
        //     10000,
        //     1
        // );

        // serialHandler.SetReadType(AuNex.Common.SerialHandler.ReadType.BYTE);
        // serialHandler.OpenSerial();
    }

    // Update is called once per frame
    void Update()
    {
        // if(node == null)
        // {
        //     node = ros2Unity.CreateNode(node_name);

        //     imu_publisher = node.CreatePublisher<sensor_msgs.msg.Imu>(imu_topic_name);
            
        //     btn_publisher = node.CreatePublisher<std_msgs.msg.Int16MultiArray>(btn_topic_name);

        //     color_subscriber = node.CreateSubscription<std_msgs.msg.ColorRGBA>(
        //         color_topic_name,
        //         ColorCallback
        //     );

        //     color = new();
        //     color.R = 255;
        //     color.G = 255;
        //     color.B = 255;
        //     color.A = 100;
        // }
        // else
        // {
        //     char[] writeData = new char[6] { (char)1, (char)1, (char)color.R, (char)color.B, (char)color.G, (char)color.A};
        //     serialHandler.WriteSerial(new String(writeData));
 

        //     Processs(serialHandler.ReadSerial().byte_data);
        // }
    }

//     private void Processs(byte[] read_data)
//     {
//         if(read_data.Length == 17)
//         {
//             var btn_msg = new std_msgs.msg.Int16MultiArray();
//             var imu_msg = new sensor_msgs.msg.Imu();

//             short[] btns = new short[] {read_data[1], read_data[2], read_data[3]};
//             btn_msg.Data = btns;

//             short accelX = ToInt16(read_data[4], read_data[5]);
//             short accelY = ToInt16(read_data[6], read_data[7]);
//             short accelZ = ToInt16(read_data[8], read_data[9]);

//             short gyroX  = ToInt16(read_data[10], read_data[11]);
//             short gyroY  = ToInt16(read_data[12], read_data[13]);
//             short gyroZ  = ToInt16(read_data[14], read_data[15]);

//             // スケールを戻す
//             float ax = accelX / 100f;
//             float ay = accelY / 100f;
//             float az = accelZ / 100f;

//             float gx = gyroX / 100f;
//             float gy = gyroY / 100f;
//             float gz = gyroZ / 100f;

//             imu_msg.Header.Frame_id = sensor_frame_id;
//             node.clock.UpdateROSClockTime(imu_msg.Header.Stamp);
//             imu_msg.Linear_acceleration.X = ax;
//             imu_msg.Linear_acceleration.Y = ay;
//             imu_msg.Linear_acceleration.Z = az;

//             imu_msg.Angular_velocity.X = gx;
//             imu_msg.Angular_velocity.Y = gy;
//             imu_msg.Angular_velocity.Z = gz;

//             btn_publisher.Publish(btn_msg);
//             imu_publisher.Publish(imu_msg);
//         }
//     }
    

//     private Int16 ToInt16(byte high, byte low)
//     {
//         return (Int16)((high << 8) | low);
//     }

//     void ColorCallback(std_msgs.msg.ColorRGBA msg)
//     {
//         color = msg;
//     }

//     void OnDestroy()
//     {
//         serialHandler.CloseSerial();
//     }
}
