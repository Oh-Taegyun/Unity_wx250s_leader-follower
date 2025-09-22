using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;
using System;

[RequireComponent(typeof(WX250sMuJoCoController))]
public class joint_state : MonoBehaviour
{
    [Header("ROS")]
    public string topicName = "/joint_states";
    public float publishHz = 50f; // 50Hz로 발행

    [Header("Debug")]
    public bool showDebugInfo = true;

    private WX250sMuJoCoController _controller;
    private ROSConnection _ros;
    private float _nextPublishTime;

    // ROS2 관절 이름 (WX250sMuJoCoController와 동일한 순서)
    private string[] _rosJointNames = {
        "waist",        // 0
        "shoulder",     // 1
        "elbow",        // 2
        "forearm_roll", // 3
        "wrist_angle",  // 4
        "wrist_rotate", // 5
        "gripper"       // 6 (Unity에서는 left_finger, right_finger로 분할되지만 ROS2에서는 1개)
    };

    void Start()
    {
        // WX250sMuJoCoController 컴포넌트 참조
        _controller = GetComponent<WX250sMuJoCoController>();
        if (_controller == null)
        {
            Debug.LogError("WX250sMuJoCoController 컴포넌트를 찾을 수 없습니다!");
            return;
        }

        // ROS 연결 설정
        _ros = ROSConnection.GetOrCreateInstance();
        _ros.RegisterPublisher<JointStateMsg>(topicName);

        // 발행 주기 설정
        _nextPublishTime = Time.time + (1f / publishHz);

        if (showDebugInfo)
        {
            Debug.Log($"Joint State Publisher 초기화 완료: {topicName} @ {publishHz}Hz");
        }
    }

    void Update()
    {
        if (_controller == null) return;

        // 발행 주기 확인
        if (Time.time < _nextPublishTime) return;
        _nextPublishTime = Time.time + (1f / publishHz);

        // 관절 상태 발행
        PublishJointStates();
    }

    void PublishJointStates()
    {
        try
        {
            // WX250sMuJoCoController에서 현재 관절 위치 가져오기
            float[] currentPositions = _controller.GetCurrentPositions();
            
            if (currentPositions == null || currentPositions.Length < 7)
            {
                Debug.LogWarning("관절 위치 데이터가 올바르지 않습니다.");
                return;
            }

            // ROS2 JointState 메시지 구성
            var header = new HeaderMsg();
            header.frame_id = "base_link"; // 또는 적절한 frame_id
            // header.stamp는 ROS2 브리지에서 자동으로 설정됨

            // Unity 관절 순서를 ROS2 관절 순서로 변환
            // Unity: [waist, shoulder, elbow, forearm_roll, wrist_angle, wrist_rotate, left_finger, right_finger]
            // ROS2:  [waist, shoulder, elbow, forearm_roll, wrist_angle, wrist_rotate, gripper]
            double[] rosPositions = new double[7];
            double[] rosVelocities = new double[7];
            double[] rosEfforts = new double[7];

            // 기본 관절 6개 (waist ~ wrist_rotate)
            for (int i = 0; i < 6; i++)
            {
                rosPositions[i] = currentPositions[i];
                rosVelocities[i] = 0.0; // 현재는 속도 정보 없음
                rosEfforts[i] = 0.0;    // 현재는 토크 정보 없음
            }

            // 그리퍼 처리 (Unity의 left_finger와 right_finger 평균값을 ROS2 gripper로)
            float gripperValue = (currentPositions[6] + currentPositions[7]) / 2f;
            rosPositions[6] = gripperValue;
            rosVelocities[6] = 0.0;
            rosEfforts[6] = 0.0;

            // JointState 메시지 생성
            var jointStateMsg = new JointStateMsg
            {
                header = header,
                name = _rosJointNames,
                position = rosPositions,
                velocity = rosVelocities,
                effort = rosEfforts
            };

            // 토픽에 발행
            _ros.Publish(topicName, jointStateMsg);

            if (showDebugInfo)
            {
                Debug.Log($"Joint States 발행: {string.Join(", ", rosPositions)}");
            }
        }
        catch (Exception ex)
        {
            Debug.LogError($"Joint States 발행 오류: {ex.Message}");
        }
    }
}