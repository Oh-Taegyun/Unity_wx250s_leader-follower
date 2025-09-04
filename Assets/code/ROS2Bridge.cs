/*
 * ROS2Bridge_ROSTCPConnector.cs - ROSTCPConnector 사용 버전
 * 
 * 이 스크립트는 Unity와 ROS2 간의 통신을 담당하는 브리지입니다.
 * Unity Robotics의 ROSTCPConnector 패키지를 사용하여 ROS2 메시지를 수신하고
 * MuJoCo 로봇 시뮬레이션에 전달합니다.
 * 
 * 주요 기능:
 * - ROS2 JointTrajectory 메시지 구독
 * - 메시지를 MuJoCo 컨트롤러에 전달
 * - 연결 상태 모니터링
 * - 자동 재연결 기능
 */

using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;  // Unity ROS2 통신 패키지
using RosMessageTypes.Sensor;          // ROS2 센서 메시지 타입
using RosMessageTypes.Trajectory;      // ROS2 궤적 메시지 타입
using RosMessageTypes.Std;             // ROS2 표준 메시지 타입

/// <summary>
/// ROS2 메시지 헤더 구조체
/// 모든 ROS2 메시지에 포함되는 공통 헤더 정보
/// </summary>
[System.Serializable]
public class Header
{
    public int sec;        // 타임스탬프 (초)
    public uint nanosec;   // 타임스탬프 (나노초)
    public string frame_id; // 좌표계 프레임 ID
}

/// <summary>
/// ROS2 JointState 메시지 구조체
/// 로봇 관절의 현재 상태 정보를 담는 메시지
/// </summary>
[System.Serializable]
public class JointStateMessage
{
    public Header header;    // 메시지 헤더
    public string[] name;    // 관절 이름 배열
    public float[] position; // 관절 위치 배열 (라디안)
    public float[] velocity; // 관절 속도 배열 (라디안/초)
    public float[] effort;   // 관절 토크 배열 (뉴턴미터)
}

/// <summary>
/// ROS2 JointTrajectory 메시지 구조체
/// 로봇 관절의 궤적 명령을 담는 메시지
/// </summary>
[System.Serializable]
public class JointTrajectoryMessage
{
    public Header header;                    // 메시지 헤더
    public string[] joint_names;             // 관절 이름 배열
    public JointTrajectoryPoint[] points;    // 궤적 포인트 배열
}

/// <summary>
/// ROS2 JointTrajectoryPoint 구조체
/// 궤적의 특정 시점에서의 관절 상태 정보
/// </summary>
[System.Serializable]
public class JointTrajectoryPoint
{
    public float[] positions;     // 관절 위치 배열 (라디안)
    public float[] velocities;    // 관절 속도 배열 (라디안/초)
    public float[] accelerations; // 관절 가속도 배열 (라디안/초²)
    public float[] effort;        // 관절 토크 배열 (뉴턴미터)
    public Duration time_from_start; // 시작점으로부터의 시간
}

/// <summary>
/// ROS2 Duration 구조체
/// 시간 간격을 나타내는 구조체
/// </summary>
[System.Serializable]
public class Duration
{
    public int sec;      // 초 단위
    public uint nanosec; // 나노초 단위
}

/// <summary>
/// ROS2Bridge 클래스
/// Unity와 ROS2 간의 통신을 담당하는 메인 브리지 클래스
/// ROSTCPConnector를 사용하여 ROS2 메시지를 수신하고 MuJoCo 시뮬레이션에 전달
/// </summary>
public class ROS2Bridge : MonoBehaviour
{
    [Header("ROS2 연결 설정")]
    [Tooltip("ROS2 서버 IP 주소 (예: localhost, 192.168.1.100)")]
    public string rosIPAddress = "localhost";
    
    [Tooltip("ROS2 서버 포트 번호 (기본값: 10000)")]
    public int rosPort = 10000;
    
    [Tooltip("구독할 ROS2 토픽 이름 (예: /leader/joint_trajectory)")]
    public string topicName = "/leader/joint_trajectory";
    
    [Header("디버그 설정")]
    [Tooltip("디버그 로그 출력 여부 (개발 시 true, 배포 시 false 권장)")]
    public bool enableDebugLog = true;
    
    // === 내부 상태 변수들 ===
    private ROSConnection ros;               // ROSTCPConnector의 ROS 연결 객체
    private bool isConnected = false;        // 현재 연결 상태를 나타내는 플래그
    private WX250sMuJoCoController mujocoController;  // MuJoCo 로봇 컨트롤러 참조
    
    // ROS2 메시지 카운터 및 연결 상태 추적
    private int receivedMessageCount = 0;    // 수신된 메시지 총 개수
    private float lastMessageTime = 0f;      // 마지막 메시지 수신 시간 (연결 상태 판단용)
    private float connectionTimeout = 5f;    // 연결 타임아웃 시간 (초)
    
    /// <summary>
    /// Unity Start 메서드
    /// 스크립트가 활성화될 때 한 번 호출됨
    /// MuJoCo 컨트롤러를 찾고 ROS2 연결을 시작함
    /// </summary>
    void Start()
    {
        Debug.Log("=== ROS2Bridge 시작됨 ===");
        
        // 같은 GameObject의 MuJoCo 컨트롤러 컴포넌트 가져오기
        // 이 컴포넌트는 실제 로봇 시뮬레이션을 제어하는 역할
        mujocoController = GetComponent<WX250sMuJoCoController>();
        if (mujocoController == null)
        {
            Debug.LogError("WX250sMuJoCoController를 찾을 수 없습니다! 같은 GameObject에 추가해주세요.");
        }
        else
        {
            Debug.Log("✅ MuJoCo 컨트롤러 가져옴");
        }
        
        // ROS2 연결 시작
        ConnectToROS2();
    }
    
    /// <summary>
    /// ROS2 ROSTCPConnector 연결 메서드
    /// ROSTCPConnector를 사용하여 ROS2 서버에 연결하고 토픽을 구독함
    /// </summary>
    public void ConnectToROS2()
    {
        try
        {
            // ROSTCPConnector의 싱글톤 인스턴스 가져오기
            // 이 객체는 Unity와 ROS2 간의 모든 통신을 관리함
            ros = ROSConnection.instance;
            
            // ROS2 서버 연결 설정
            ros.RosIPAddress = rosIPAddress;  // 서버 IP 주소 설정
            ros.RosPort = rosPort;            // 서버 포트 설정
            
            // 연결 상태 모니터링 코루틴 시작
            // 메시지 수신 시간을 기반으로 연결 상태를 추적함
            StartCoroutine(MonitorConnection());
            
            // 지정된 토픽 구독 시작
            SubscribeToTopic();
            
            if (enableDebugLog)
                Debug.Log($"🔗 ROS2 Bridge 연결 시도: {rosIPAddress}:{rosPort}");
        }
        catch (Exception ex)
        {
            Debug.LogError($"❌ ROS2 Bridge 연결 실패: {ex.Message}");
        }
    }
    
    /// <summary>
    /// ROS 연결 상태 모니터링 코루틴
    /// 메시지 수신 시간을 기반으로 연결 상태를 추정하고 로그를 출력함
    /// 0.5초마다 연결 상태를 확인하여 연결/해제 상태 변화를 감지함
    /// </summary>
    IEnumerator MonitorConnection()
    {
        while (true)
        {
            bool wasConnected = isConnected;  // 이전 연결 상태 저장
            
            // 메시지 수신 시간을 기반으로 연결 상태 판단
            // 최근 메시지 수신 시간이 타임아웃을 초과하면 연결 끊어짐으로 판단
            float timeSinceLastMessage = Time.time - lastMessageTime;
            isConnected = (ros != null) && (receivedMessageCount > 0) && (timeSinceLastMessage < connectionTimeout);
            
            // 연결 상태가 변경되었을 때만 로그 출력
            if (wasConnected != isConnected)
            {
                if (isConnected)
                {
                    if (enableDebugLog)
                        Debug.Log("✅ ROS2 Bridge 연결 성공!");
                }
                else
                {
                    if (enableDebugLog)
                        Debug.LogWarning($"⚠️ ROS2 Bridge 연결 끊어짐 (마지막 메시지: {timeSinceLastMessage:F1}초 전)");
                }
            }
            
            yield return new WaitForSeconds(0.5f); // 0.5초마다 연결 상태 확인
        }
    }
    
    /// <summary>
    /// ROS2 토픽 구독 요청 메서드
    /// ROSTCPConnector를 사용하여 JointTrajectory 토픽을 구독함
    /// 메시지가 수신되면 OnTrajectoryMessage 콜백이 자동으로 호출됨
    /// </summary>
    void SubscribeToTopic()
    {
        if (ros == null) 
        {
            Debug.LogError("ROS 연결 객체가 null입니다!");
            return;
        }
        
        // JointTrajectory 토픽 구독
        // 제네릭 타입으로 메시지 타입을 지정하고, 콜백 함수를 등록함
        ros.Subscribe<JointTrajectoryMsg>(topicName, OnTrajectoryMessage);
        
        if (enableDebugLog)
            Debug.Log($"📡 토픽 구독 요청: {topicName}");
    }
    
    /// <summary>
    /// ROS2 JointTrajectory 메시지 수신 처리 콜백
    /// ROSTCPConnector에서 전달받은 메시지를 MuJoCo 컨트롤러에 전달함
    /// 이 메서드는 ROS2 메시지가 수신될 때마다 자동으로 호출됨
    /// </summary>
    /// <param name="msg">수신된 ROS2 JointTrajectory 메시지</param>
    void OnTrajectoryMessage(JointTrajectoryMsg msg)
    {
        try
        {
            // 메시지 수신 통계 업데이트
            receivedMessageCount++;           // 수신 메시지 카운터 증가
            lastMessageTime = Time.time;      // 마지막 수신 시간 업데이트 (연결 상태 판단용)
            
            if (enableDebugLog)
                Debug.Log($"🎯 궤적 명령 수신: {msg.joint_names.Length}개 관절 (메시지 #{receivedMessageCount})");
            
            // MuJoCo 컨트롤러에 궤적 명령 전달
            if (mujocoController != null)
            {
                // ROS2 메시지를 내부 형식으로 변환
                var trajectoryData = ConvertToTrajectoryMessage(msg);
                
                // MuJoCo 시뮬레이션에 궤적 명령 실행
                mujocoController.ExecuteTrajectory(trajectoryData);
                
                if (enableDebugLog)
                    Debug.Log("✅ MuJoCo 컨트롤러에 궤적 전달 완료");
            }
            else
            {
                Debug.LogWarning("⚠️ MuJoCo 컨트롤러가 null입니다!");
            }
        }
        catch (Exception ex)
        {
            Debug.LogError($"❌ 메시지 처리 오류: {ex.Message}");
        }
    }
    
    /// <summary>
    /// ROS2 JointTrajectoryMsg를 내부 JointTrajectoryMessage로 변환하는 메서드
    /// ROS2 메시지 형식을 Unity 내부에서 사용하는 형식으로 변환함
    /// double 배열을 float 배열로 변환하는 과정도 포함됨
    /// </summary>
    /// <param name="rosMsg">변환할 ROS2 JointTrajectoryMsg</param>
    /// <returns>변환된 내부 JointTrajectoryMessage</returns>
    JointTrajectoryMessage ConvertToTrajectoryMessage(JointTrajectoryMsg rosMsg)
    {
        // 궤적 포인트 배열 변환
        var points = new JointTrajectoryPoint[rosMsg.points.Length];
        for (int i = 0; i < rosMsg.points.Length; i++)
        {
            points[i] = new JointTrajectoryPoint
            {
                // double 배열을 float 배열로 변환 (Unity는 float를 주로 사용)
                positions = ConvertDoubleArrayToFloatArray(rosMsg.points[i].positions),
                velocities = ConvertDoubleArrayToFloatArray(rosMsg.points[i].velocities),
                accelerations = ConvertDoubleArrayToFloatArray(rosMsg.points[i].accelerations),
                effort = ConvertDoubleArrayToFloatArray(rosMsg.points[i].effort),
                time_from_start = new Duration
                {
                    sec = rosMsg.points[i].time_from_start.sec,
                    nanosec = rosMsg.points[i].time_from_start.nanosec
                }
            };
        }
        
        // 최종 메시지 객체 생성
        return new JointTrajectoryMessage
        {
            header = new Header
            {
                sec = rosMsg.header.stamp.sec,
                nanosec = rosMsg.header.stamp.nanosec,
                frame_id = rosMsg.header.frame_id
            },
            joint_names = rosMsg.joint_names,  // 관절 이름 배열 (변환 불필요)
            points = points                    // 변환된 궤적 포인트 배열
        };
    }
    
    /// <summary>
    /// double 배열을 float 배열로 변환하는 헬퍼 메서드
    /// ROS2는 double 정밀도를 사용하지만 Unity는 float를 주로 사용하므로 변환이 필요함
    /// </summary>
    /// <param name="doubleArray">변환할 double 배열</param>
    /// <returns>변환된 float 배열 (null 입력 시 null 반환)</returns>
    float[] ConvertDoubleArrayToFloatArray(double[] doubleArray)
    {
        if (doubleArray == null) return null;
        
        // 새로운 float 배열 생성
        float[] floatArray = new float[doubleArray.Length];
        
        // 각 요소를 double에서 float로 캐스팅
        for (int i = 0; i < doubleArray.Length; i++)
        {
            floatArray[i] = (float)doubleArray[i];
        }
        
        return floatArray;
    }
    
    /// <summary>
    /// GameObject 파괴 시 ROS 연결 정리 메서드
    /// Unity에서 GameObject가 파괴될 때 자동으로 호출됨
    /// ROSTCPConnector는 자동으로 연결을 관리하므로 별도 정리 작업은 불필요함
    /// </summary>
    void OnDestroy()
    {
        if (ros != null)
        {
            // ROSTCPConnector는 자동으로 연결을 관리하므로 별도 정리 불필요
            if (enableDebugLog)
                Debug.Log("🧹 ROS2 Bridge 정리 완료");
        }
    }
    
    /// <summary>
    /// 애플리케이션 종료 시 ROS 연결 정리 메서드
    /// Unity 애플리케이션이 종료될 때 자동으로 호출됨
    /// </summary>
    void OnApplicationQuit()
    {
        if (enableDebugLog)
            Debug.Log("👋 ROS2 Bridge 애플리케이션 종료");
    }
    
    // === 공개 API 메서드들 ===
    // 다른 스크립트에서 ROS2Bridge의 상태를 확인하거나 제어할 수 있는 메서드들
    
    /// <summary>
    /// 수신된 메시지 수를 반환하는 공개 메서드
    /// 디버깅이나 통계 수집에 사용됨
    /// </summary>
    /// <returns>현재까지 수신된 메시지의 총 개수</returns>
    public int GetReceivedMessageCount()
    {
        return receivedMessageCount;
    }
    
    /// <summary>
    /// ROS 연결 상태를 반환하는 공개 메서드
    /// 다른 스크립트에서 연결 상태를 확인할 때 사용됨
    /// </summary>
    /// <returns>현재 ROS2 서버와의 연결 상태 (true: 연결됨, false: 연결 안됨)</returns>
    public bool IsConnected()
    {
        return isConnected;
    }
    
    /// <summary>
    /// ROS 연결을 재시도하는 공개 메서드
    /// 연결이 끊어졌을 때 수동으로 재연결을 시도할 때 사용됨
    /// </summary>
    public void Reconnect()
    {
        if (ros != null)
        {
            ros.Disconnect();  // 기존 연결 해제
            ConnectToROS2();   // 새로 연결 시도
        }
    }
}
