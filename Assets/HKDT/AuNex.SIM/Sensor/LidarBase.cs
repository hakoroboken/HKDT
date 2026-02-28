using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using ROS2;
using Unity.VisualScripting;

public class LidarBase : MonoBehaviour
{
    // ===== Lidar Settings =====
    public bool enable_debug = false;
    public string node_name = "lidar_node";
    public string topic_name = "/scan";
    public string frame_id = "lidar_frame";
    public float scan_frequency = 10.0f; // Hz
    public float max_distance = 10.0f; // meters
    public float min_distance = 0.1f; // meters
    public float max_angle = 360.0f; // degrees
    // 点群のノイズを決める
    public float noise_stddev = 0.01f; // meters
    public int num_points = 360;
    public LayerMask obstacle_layer;

    // ===== ROS2 =====
    private ROS2UnityComponent ros2Unity;
    private ROS2Node node;
    private IPublisher<sensor_msgs.msg.LaserScan> publisher;

    private float elapsed_time;


    // Start is called before the first frame update
    void Start()
    {
        ros2Unity = GetComponent<ROS2UnityComponent>();

        elapsed_time = 0.0f;
    }

    // Update is called once per frame
    void Update()
    {
        if(ros2Unity.Ok())
        {
            if(node == null)
            {
                node = ros2Unity.CreateNode(node_name);
                publisher = node.CreatePublisher<sensor_msgs.msg.LaserScan>(topic_name);
            }
            else
            {
                elapsed_time += Time.deltaTime;

                if(elapsed_time >= (1.0f / scan_frequency))
                {
                    elapsed_time = 0.0f;
                    ScanAndPublish();
                }
            }
        }
    }

    void ScanAndPublish()
    {
        float[] distance = new float[num_points];

        for(int i = 0; i < num_points; i++)
        {
            float angle = max_angle / num_points * i;
            Vector3 up = transform.up;
            up.x = 0;
            up.z = 0;
            Vector3 direction = Quaternion.AngleAxis(angle, up) * transform.forward;

            Vector3 origin = transform.position;
            Ray ray = new(origin, direction);
            RaycastHit hit = new();

            bool hitResult = Physics.Raycast(ray, out hit, max_distance, obstacle_layer);

            if(hitResult)
            {
                if(hit.distance > max_distance)
                {
                    distance[num_points - 1 - i] = Mathf.Infinity;
                }
                else
                {
                    distance[num_points - 1 - i] = hit.distance * Random.Range(1.0f - noise_stddev, 1.0f + noise_stddev);
                }
            }
            else
            {
                distance[num_points - 1 - i] = Mathf.Infinity;
            }

            if(enable_debug)
            {
                if(hitResult)
                {
                    Debug.DrawRay(ray.origin, ray.direction * hit.distance, Color.green, 0.1f);
                }
                else
                {
                    Debug.DrawRay(ray.origin, ray.direction * max_distance, Color.red, 0.1f);
                }
            }
        }

        // Publish LaserScan message
        sensor_msgs.msg.LaserScan msg = new sensor_msgs.msg.LaserScan();
        msg.Header.Frame_id = frame_id;
        msg.Angle_min = -30.0f * Mathf.Deg2Rad;
        msg.Angle_max = Mathf.Deg2Rad * 210.0f;
        msg.Angle_increment = (msg.Angle_max - msg.Angle_min) / num_points;
        msg.Time_increment = 0.0f;
        msg.Scan_time = 1.0f / scan_frequency;
        msg.Range_min = min_distance;
        msg.Range_max = max_distance;
        msg.Ranges = distance;

        publisher.Publish(msg);
    }
}
