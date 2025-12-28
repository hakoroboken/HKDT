// Unityの基本的なusing
using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

// 行列計算ライブラリ
using MathNet.Numerics.LinearAlgebra;

// ROS2を使うためのusing
using ROS2;
// AuNexのローカライゼーションとアルゴリズムのライブラリ
using AuNex.Localization;
using AuNex.Algorithm;

/// <summary>
/// ICPの使用例
/// </summary>
public class IcpExample : MonoBehaviour
{
    // ICPのパラメータ
    public int max_iterations = 50;
    public float tolerance = 0.001f;
    // Hashセルサイズ
    public float hash_cell_size = 0.1f;
    // ICPの最大対応点距離
    public float max_correspondence_distance = 0.3f;
    // ノードの名前
    public String node_name = "icp_example_node";
    // LIDARのトピック名
    public String scan_topic = "/scan";
    // 姿勢のトピック名
    public String posture_topic = "/robot_posture";
    // デバッグ用で動かすオドメトリのゲームオブジェクト
    public GameObject odometry;


    // ROS2
    // 絶対に必要なもの
    private ROS2UnityComponent ros2Unity;
    // ROS2ノード
    private ROS2Node node;
    // LIDARのサブスクライバ
    private ISubscription<sensor_msgs.msg.LaserScan> scan_subscription;
    // 姿勢のサブスクライバ
    private ISubscription<geometry_msgs.msg.Quaternion> posture_subscription;
    // AuNex.Localizationライブラリで定義されたICPアルゴリズム
    private ICP_Hash icp;
    // ICPの初期化フラグ
    private bool is_initialized = false;
    // 推定結果
    // 回転は２次元なのでradで表現
    private float rotation_;
    // 並進は2Dベクトルで表現
    private Vector2 translation_;



    // Start is called before the first frame update
    void Start()
    {
        ros2Unity = GetComponent<ROS2UnityComponent>();
        icp = new ICP_Hash(hash_cell_size, max_correspondence_distance);
        rotation_ = 0.0f;
        translation_ = Vector2.zero;
    }

    // Update is called once per frame
    void Update()
    {
        if(node == null)
        {
            node = ros2Unity.CreateNode(node_name);
            
            scan_subscription = node.CreateSubscription<sensor_msgs.msg.LaserScan>
            (
                scan_topic, 
                ProcessScan
            );

            posture_subscription = node.CreateSubscription<geometry_msgs.msg.Quaternion>
            (
                posture_topic,
                ProcessPosture
            );
        }

        // オドメトリのゲームオブジェクトを更新（デバッグ用）
        odometry.transform.position = new Vector3((float)translation_.y, 0.25f, (float)translation_.x);
        odometry.transform.rotation = MathUtils.ToQuatUnity(rotation_);
    }

    void ProcessScan(sensor_msgs.msg.LaserScan msg)
    {
        // レーザースキャンを点群に変換
        List<Vector2> scan_points = new(msg.Ranges.Length);
        ScanUtils.LaserScanToPointCloud(msg, ref scan_points);

        if(!is_initialized)
        {
            icp.SetTargetPoints(scan_points);
            is_initialized = true;
            return;
        }

        // ICPの初期推定を設定
        icp.SetInitialPose(translation_, rotation_);

        // ICPで位置合わせ
        bool is_converged = icp.Align(scan_points, max_iterations, tolerance);

        if(is_converged)
        {
            // 推定結果を取得
            translation_ = icp.GetEstimatedPosition();
            rotation_ = icp.GetEstimatedRotation();
        }
        else
        {
            Debug.LogWarning("ICPが収束しませんでした。");
            return;
        }        
    }

    void ProcessPosture(geometry_msgs.msg.Quaternion msg)
    {
        // 受信した姿勢を使ってICPの初期推定を更新
        Quaternion quat = new Quaternion((float)msg.X, (float)msg.Y, (float)msg.Z, (float)msg.W);
        rotation_ = MathUtils.toRadUnity(quat);
    }
}
