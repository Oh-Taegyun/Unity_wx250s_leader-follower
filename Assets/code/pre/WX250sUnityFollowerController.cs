/*
 * WX250sUnityFollowerController.cs
 * wx250s957 로봇팔의 Unity 팔로워 시스템을 통합 관리하는 메인 컨트롤러
 * 
 * 이 스크립트는 ROS2Bridge, MuJoCoController, JointMapper를 조율하여 전체 시스템을 제어합니다.
 * 모든 컴포넌트를 통합 관리하고 시스템 상태를 모니터링하며 사용자 인터페이스를 제공합니다.
 * 
 * 주요 기능:
 * - 전체 시스템 초기화 및 컴포넌트 자동 할당
 * - ROS2 연결 상태 모니터링 및 자동 재연결
 * - 시스템 상태 실시간 추적 및 관리
 * - 그리퍼 제어 및 동기화 관리
 * - 런타임 GUI를 통한 시스템 제어
 * - 로봇 리셋 및 설정 관리
 * 
 * 사용 예시:
 * - ROS2-Unity 통합 로봇 시뮬레이션 시스템
 * - 실시간 로봇 팔 제어 및 모니터링
 * - 그리퍼 제어 및 테스트
 * - 시스템 상태 진단 및 디버깅
 */

using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// wx250s957 로봇팔의 Unity 팔로워 시스템 통합 컨트롤러
/// 
/// 이 클래스는 ROS2Bridge, MuJoCoController, JointMapper를 조율하여 전체 시스템을 관리합니다.
/// 모든 컴포넌트를 통합 관리하고 시스템 상태를 모니터링하며 사용자 인터페이스를 제공합니다.
/// 
/// 주요 역할:
/// - 전체 시스템 초기화 및 컴포넌트 자동 할당
/// - ROS2 연결 상태 모니터링 및 자동 재연결
/// - 시스템 상태 실시간 추적 및 관리
/// - 그리퍼 제어 및 동기화 관리
/// - 런타임 GUI를 통한 시스템 제어
/// </summary>
public class WX250sUnityFollowerController : MonoBehaviour
{
    [Header("컴포넌트 참조")]
    [Tooltip("ROS2 연결 브리지 컴포넌트 (ROS2-Unity 통신 담당)")]
    public ROS2Bridge ros2Bridge;
    
    [Tooltip("MuJoCo 시뮬레이션 컨트롤러 (로봇 팔 제어 담당)")]
    public WX250sMuJoCoController mujocoController;
    
    [Tooltip("관절 매핑 관리자 (ROS2 ↔ Unity MuJoCo 데이터 변환)")]
    public UnityJointMapper jointMapper;
    
    [Header("시스템 설정")]
    [Tooltip("자동 초기화 활성화 (Start()에서 자동으로 시스템 초기화)")]
    public bool autoInitialize = true;
    
    [Tooltip("ROS2 연결 타임아웃 (초, 연결 시도 최대 대기 시간)")]
    public float connectionTimeout = 10f;
    
    [Tooltip("시스템 업데이트 주기 (Hz, 높을수록 정밀하지만 CPU 부하 증가)")]
    public float updateRate = 100f;
    
    [Header("그리퍼 설정")]
    [Tooltip("그리퍼 제어 활성화 (그리퍼 관련 기능 사용 여부)")]
    public bool enableGripperControl = true;
    
    [Tooltip("그리퍼 테스트 값 (0~1, 테스트 버튼 클릭 시 사용할 값)")]
    public float gripperTestValue = 0.5f;
    
    [Header("시스템 상태 (런타임에서 자동 업데이트)")]
    [Tooltip("ROS2 연결 상태 (true: 연결됨, false: 연결 안됨)")]
    public bool isConnected = false;
    
    [Tooltip("시스템 초기화 완료 여부 (모든 컴포넌트 초기화 완료)")]
    public bool isInitialized = false;
    
    [Tooltip("마지막 업데이트 시간 (Time.time 기준)")]
    public float lastUpdateTime = 0f;
    
    [Tooltip("수신된 메시지 수 (ROS2에서 받은 총 메시지 개수)")]
    public int receivedMessageCount = 0;
    
    [Header("디버그 설정")]
    [Tooltip("디버그 로그 출력 여부 (개발 시 true, 배포 시 false 권장)")]
    public bool enableDebugLog = true;
    
    [Tooltip("상태 GUI 표시 여부 (런타임에서 시스템 상태 표시)")]
    public bool showStatusGUI = true;
    
    [Tooltip("그리퍼 제어 GUI 표시 여부 (런타임에서 그리퍼 제어 버튼 표시)")]
    public bool showGripperControls = true;
    
    // === 내부 상태 변수들 ===
    private float connectionStartTime;        // 연결 시작 시간 (Time.time 기준)
    private bool connectionAttempted = false; // 연결 시도 여부 (중복 시도 방지)
    private float lastGripperValue = 0f;      // 마지막 그리퍼 값 (변화 감지용)
    
    /// <summary>
    /// Unity Start 메서드
    /// 스크립트가 활성화될 때 한 번 호출되어 자동 초기화를 수행함
    /// </summary>
    void Start()
    {
        if (autoInitialize)
        {
            InitializeSystem();  // 자동 초기화 활성화 시 시스템 초기화
        }
    }
    
    /// <summary>
    /// 팔로워 시스템 전체 초기화 메서드
    /// 
    /// 모든 컴포넌트를 자동 할당하고 연결을 시도합니다.
    /// 이 메서드는 시스템의 핵심 초기화 로직을 담당하며,
    /// 필수 컴포넌트들의 존재를 확인하고 ROS2 연결을 시작합니다.
    /// 
    /// 초기화 과정:
    /// 1. 컴포넌트 자동 할당 (같은 GameObject에서 찾기)
    /// 2. 필수 컴포넌트 존재 확인
    /// 3. ROS2 연결 시도
    /// 4. 초기화 완료 플래그 설정
    /// </summary>
    public void InitializeSystem()
    {
        if (enableDebugLog)
            Debug.Log("WX250s Unity 팔로워 시스템 초기화 시작...");
        
        // === 컴포넌트 자동 할당 ===
        // Inspector에서 수동으로 할당하지 않은 경우 같은 GameObject에서 자동으로 찾기
        if (ros2Bridge == null)
            ros2Bridge = GetComponent<ROS2Bridge>();
        
        if (mujocoController == null)
            mujocoController = GetComponent<WX250sMuJoCoController>();
        
        if (jointMapper == null)
            jointMapper = GetComponent<UnityJointMapper>();
        
        // === 필수 컴포넌트 존재 확인 ===
        if (ros2Bridge == null)
        {
            Debug.LogError("ROS2Bridge 컴포넌트를 찾을 수 없습니다! 같은 GameObject에 추가해주세요.");
            return;
        }
        
        if (mujocoController == null)
        {
            Debug.LogError("WX250sMuJoCoController 컴포넌트를 찾을 수 없습니다! 같은 GameObject에 추가해주세요.");
            return;
        }
        
        if (jointMapper == null)
        {
            Debug.LogError("UnityJointMapper 컴포넌트를 찾을 수 없습니다! 같은 GameObject에 추가해주세요.");
            return;
        }
        
        // === ROS2 연결 시도 ===
        StartCoroutine(WaitForConnection());
        
        isInitialized = true;
        
        if (enableDebugLog)
            Debug.Log("WX250s Unity 팔로워 시스템 초기화 완료!");
    }
    
    /// <summary>
    /// ROS2 연결 대기 코루틴 메서드
    /// 
    /// 타임아웃 시간 내에 ROS2 연결이 성공하는지 확인합니다.
    /// 0.1초마다 연결 상태를 확인하여 연결 성공 시 즉시 반환하고,
    /// 타임아웃 시 경고 메시지를 출력합니다.
    /// 
    /// 연결 확인 과정:
    /// 1. 연결 시작 시간 기록
    /// 2. 0.1초마다 ROS2Bridge.IsConnected() 호출
    /// 3. 연결 성공 시 즉시 반환
    /// 4. 타임아웃 시 경고 메시지 출력
    /// </summary>
    IEnumerator WaitForConnection()
    {
        connectionStartTime = Time.time;  // 연결 시작 시간 기록
        connectionAttempted = true;       // 연결 시도 플래그 설정
        
        // 타임아웃 시간 동안 연결 상태 확인
        while (Time.time - connectionStartTime < connectionTimeout)
        {
            // ROS2 연결 상태 확인
            if (ros2Bridge != null && ros2Bridge.IsConnected())
            {
                isConnected = true;
                if (enableDebugLog)
                    Debug.Log("ROS2 연결 성공!");
                break;
            }
            
            yield return new WaitForSeconds(0.1f); // 0.1초마다 확인
        }
        
        // 타임아웃 시 경고 메시지
        if (!isConnected)
        {
            Debug.LogWarning($"ROS2 연결 시간 초과 ({connectionTimeout}초)");
        }
    }
    
    /// <summary>
    /// Unity Update 루프 - 시스템 상태 모니터링 및 관리
    /// 
    /// 매 프레임마다 시스템 상태를 모니터링하고 관리합니다.
    /// 초기화가 완료된 후에만 실행되며, 다음 작업들을 순차적으로 수행합니다:
    /// 1. 연결 상태 모니터링
    /// 2. 시스템 상태 업데이트
    /// 3. 그리퍼 상태 모니터링
    /// </summary>
    void Update()
    {
        if (!isInitialized) return;
        
        // 1) 연결 상태 모니터링
        MonitorConnection();
        
        // 2) 시스템 상태 업데이트
        UpdateSystemStatus();
        
        // 3) 그리퍼 상태 모니터링
        MonitorGripperStatus();
    }
    
    /// <summary>
    /// ROS2 연결 상태 모니터링 메서드
    /// 
    /// ROS2 연결 상태 변화를 감지하고 로그를 출력합니다.
    /// 연결이 복구되거나 끊어질 때마다 상태 변화를 감지하여
    /// 사용자에게 알림을 제공합니다.
    /// </summary>
    void MonitorConnection()
    {
        if (ros2Bridge != null)
        {
            bool wasConnected = isConnected;  // 이전 연결 상태 저장
            isConnected = ros2Bridge.IsConnected();  // 현재 연결 상태 확인
            
            // 연결 상태 변화 감지
            if (wasConnected != isConnected)
            {
                if (isConnected)
                {
                    if (enableDebugLog)
                        Debug.Log("ROS2 연결 복구됨");
                }
                else
                {
                    if (enableDebugLog)
                        Debug.LogWarning("ROS2 연결 끊어짐");
                }
            }
        }
    }
    
    /// <summary>
    /// 시스템 상태 업데이트 메서드
    /// 
    /// 시스템의 기본 상태 정보를 업데이트합니다.
    /// 마지막 업데이트 시간을 갱신하여 시스템이 정상적으로
    /// 작동하고 있음을 추적합니다.
    /// </summary>
    void UpdateSystemStatus()
    {
        lastUpdateTime = Time.time;  // 마지막 업데이트 시간 갱신
    }
    
    /// <summary>
    /// 그리퍼 상태 모니터링 메서드
    /// 
    /// 그리퍼 값의 변화를 감지하고 로그를 출력합니다.
    /// 0.01 이상의 변화가 있을 때만 로그를 출력하여
    /// 불필요한 로그 스팸을 방지합니다.
    /// </summary>
    void MonitorGripperStatus()
    {
        if (mujocoController != null && enableGripperControl)
        {
            float currentGripper = mujocoController.GetGripperValue();
            // 0.01 이상 변화 시에만 로그 출력 (로그 스팸 방지)
            if (Mathf.Abs(currentGripper - lastGripperValue) > 0.01f)
            {
                lastGripperValue = currentGripper;
                if (enableDebugLog)
                    Debug.Log($"그리퍼 값 변경: {currentGripper:F3}");
            }
        }
    }
    
    // === 공개 API 메서드들 ===
    // 다른 스크립트에서 WX250sUnityFollowerController의 기능을 사용할 수 있는 메서드들
    
    /// <summary>
    /// ROS2 연결 재시도 메서드
    /// 
    /// ROS2 연결이 끊어진 경우 수동으로 재연결을 시도합니다.
    /// ROS2Bridge의 ConnectToROS2()를 호출하고 연결 대기 코루틴을 시작합니다.
    /// </summary>
    public void RestartConnection()
    {
        if (ros2Bridge != null)
        {
            ros2Bridge.ConnectToROS2();  // ROS2 연결 시도
            StartCoroutine(WaitForConnection());  // 연결 대기 코루틴 시작
        }
    }
    
    /// <summary>
    /// 로봇을 초기 위치로 리셋하는 메서드
    /// 
    /// 모든 관절을 0 위치로 설정하여 로봇을 초기 상태로 되돌립니다.
    /// 안전한 위치로 로봇을 이동시키거나 테스트를 위해 사용됩니다.
    /// 
    /// 리셋 과정:
    /// 1. 8개 관절 모두 0 위치로 설정
    /// 2. MuJoCo 컨트롤러에 리셋 위치 적용
    /// 3. 리셋 완료 로그 출력
    /// </summary>
    public void ResetRobot()
    {
        if (mujocoController != null)
        {
            // 모든 관절을 0 위치로 리셋
            float[] resetPositions = new float[8];  // 8개 관절 (Unity MuJoCo 구조)
            for (int i = 0; i < resetPositions.Length; i++)
            {
                resetPositions[i] = 0f;  // 모든 관절을 0 라디안으로 설정
            }
            
            // 각 관절에 리셋 위치 적용
            for (int i = 0; i < mujocoController.jointNames.Length; i++)
            {
                mujocoController.SetJointPosition(i, resetPositions[i]);
            }
            
            if (enableDebugLog)
                Debug.Log("로봇 위치 리셋 완료");
        }
    }
    
    /// <summary>
    /// 시스템 업데이트 주기를 설정하는 메서드
    /// 
    /// 시스템의 업데이트 주기를 설정하고 MuJoCo 컨트롤러에도 동일한 주기를 적용합니다.
    /// 높은 주기는 더 정밀한 제어를 제공하지만 CPU 부하가 증가합니다.
    /// </summary>
    /// <param name="rate">업데이트 주기 (Hz, 기본값: 100)</param>
    public void SetUpdateRate(float rate)
    {
        updateRate = rate;
        if (mujocoController != null)
        {
            mujocoController.updateRate = rate;  // MuJoCo 컨트롤러에도 동일한 주기 적용
        }
    }
    
    /// <summary>
    /// 부드러운 움직임을 활성화/비활성화하는 메서드
    /// 
    /// 로봇의 움직임을 부드럽게 만들지 여부를 설정합니다.
    /// 활성화하면 급격한 움직임이 방지되어 자연스러운 애니메이션이 구현됩니다.
    /// </summary>
    /// <param name="enable">부드러운 움직임 활성화 여부 (true: 활성화, false: 비활성화)</param>
    public void EnableSmoothing(bool enable)
    {
        if (mujocoController != null)
        {
            mujocoController.enableSmoothing = enable;
        }
    }
    
    /// <summary>
    /// 부드러운 움직임 보간 계수를 설정하는 메서드
    /// 
    /// 부드러운 움직임의 속도를 조절합니다.
    /// 높은 값일수록 빠르게 목표 위치에 도달하고, 낮은 값일수록 천천히 움직입니다.
    /// </summary>
    /// <param name="factor">보간 계수 (0~1, 0: 매우 천천히, 1: 즉시 도달)</param>
    public void SetSmoothingFactor(float factor)
    {
        if (mujocoController != null)
        {
            mujocoController.smoothingFactor = factor;
        }
    }
    
    // === 그리퍼 제어 메서드들 ===
    // 그리퍼 관절의 특별한 처리를 위한 메서드들
    
    /// <summary>
    /// 그리퍼 값을 설정하는 메서드
    /// 
    /// 그리퍼의 열림/닫힘 상태를 설정합니다.
    /// 0: 완전히 열림, 1: 완전히 닫힘
    /// 그리퍼 제어가 활성화된 경우에만 동작합니다.
    /// </summary>
    /// <param name="value">그리퍼 값 (0~1, 0: 열림, 1: 닫힘)</param>
    public void SetGripperValue(float value)
    {
        if (mujocoController != null && enableGripperControl)
        {
            mujocoController.SetGripperValue(value);
            if (enableDebugLog)
                Debug.Log($"그리퍼 값 설정: {value:F3}");
        }
    }
    
    /// <summary>
    /// 그리퍼를 열는 메서드
    /// 
    /// 그리퍼를 완전히 열린 상태(0)로 설정합니다.
    /// 물건을 놓거나 그리퍼를 열어야 할 때 사용됩니다.
    /// </summary>
    public void OpenGripper()
    {
        SetGripperValue(0f);  // 완전히 열림
    }
    
    /// <summary>
    /// 그리퍼를 닫는 메서드
    /// 
    /// 그리퍼를 완전히 닫힌 상태(1)로 설정합니다.
    /// 물건을 잡거나 그리퍼를 닫아야 할 때 사용됩니다.
    /// </summary>
    public void CloseGripper()
    {
        SetGripperValue(1f);  // 완전히 닫힘
    }
    
    /// <summary>
    /// 그리퍼 동기화를 토글하는 메서드
    /// 
    /// MuJoCo 컨트롤러와 JointMapper 모두에 동기화 설정을 적용합니다.
    /// 동기화가 활성화되면 left_finger와 right_finger가 항상 동일한 값을 가집니다.
    /// </summary>
    public void ToggleGripperSync()
    {
        if (mujocoController != null)
        {
            bool currentSync = mujocoController.enableGripperSync;
            mujocoController.SetGripperSync(!currentSync);  // MuJoCo 컨트롤러 동기화 토글
            
            // JointMapper에도 동일한 설정 적용
            if (jointMapper != null)
            {
                jointMapper.SetGripperSync(!currentSync);  // JointMapper 동기화 토글
            }
            
            if (enableDebugLog)
                Debug.Log($"그리퍼 동기화: {(!currentSync ? "ON" : "OFF")}");
        }
    }
    
    // === 상태 정보 반환 ===
    // 시스템의 현재 상태를 확인할 수 있는 메서드들
    
    /// <summary>
    /// 현재 시스템 상태 정보를 반환하는 메서드
    /// 
    /// 시스템의 모든 주요 상태 정보를 SystemStatus 구조체로 반환합니다.
    /// 다른 스크립트에서 시스템 상태를 확인하거나 모니터링할 때 사용됩니다.
    /// </summary>
    /// <returns>시스템 상태 구조체 (연결 상태, 초기화 상태, 메시지 수, 그리퍼 값 등)</returns>
    public SystemStatus GetSystemStatus()
    {
        return new SystemStatus
        {
            isConnected = isConnected,  // ROS2 연결 상태
            isInitialized = isInitialized,  // 시스템 초기화 완료 여부
            receivedMessageCount = receivedMessageCount,  // 수신된 메시지 수
            lastUpdateTime = lastUpdateTime,  // 마지막 업데이트 시간
            connectionUptime = isConnected ? Time.time - connectionStartTime : 0f,  // 연결 지속 시간
            currentGripperValue = mujocoController != null ? mujocoController.GetGripperValue() : 0f  // 현재 그리퍼 값
        };
    }
    
    /// <summary>
    /// Unity GUI를 통한 시스템 상태 표시 및 제어 메서드
    /// 
    /// 런타임에서 시스템 상태를 모니터링하고 제어할 수 있는 GUI를 제공합니다.
    /// 화면 좌측 하단에 시스템 정보와 제어 버튼들을 표시합니다.
    /// 
    /// 표시 정보:
    /// - 시스템 상태 (초기화, 연결, 메시지 수, 업데이트 시간)
    /// - 시스템 제어 버튼 (연결 재시도, 로봇 리셋)
    /// - 그리퍼 제어 버튼 (열기, 닫기, 테스트, 동기화 토글)
    /// </summary>
    void OnGUI()
    {
        if (!showStatusGUI) return;
        
        // GUI 영역 설정 (화면 좌측 하단)
        GUILayout.BeginArea(new Rect(10, 470, 400, 300));
        GUILayout.Label("WX250s Unity Follower System", GUI.skin.box);
        
        // === 시스템 상태 정보 표시 ===
        GUILayout.Label($"초기화 상태: {(isInitialized ? "완료" : "미완료")}");
        GUILayout.Label($"ROS2 연결: {(isConnected ? "연결됨" : "연결 끊어짐")}");
        GUILayout.Label($"수신 메시지: {receivedMessageCount}개");
        GUILayout.Label($"마지막 업데이트: {Time.time - lastUpdateTime:F1}초 전");
        
        if (isConnected)
        {
            GUILayout.Label($"연결 시간: {Time.time - connectionStartTime:F1}초");
        }
        
        GUILayout.Space(10);
        
        // === 시스템 제어 버튼들 ===
        if (GUILayout.Button("연결 재시도"))
        {
            RestartConnection();
        }
        
        if (GUILayout.Button("로봇 리셋"))
        {
            ResetRobot();
        }
        
        // === 그리퍼 제어 섹션 ===
        if (showGripperControls && enableGripperControl)
        {
            GUILayout.Space(10);
            GUILayout.Label("그리퍼 제어:");
            
            // 그리퍼 제어 버튼들 (가로 배치)
            GUILayout.BeginHorizontal();
            if (GUILayout.Button("열기"))
            {
                OpenGripper();
            }
            
            if (GUILayout.Button("닫기"))
            {
                CloseGripper();
            }
            
            if (GUILayout.Button("테스트"))
            {
                SetGripperValue(gripperTestValue);
            }
            GUILayout.EndHorizontal();
            
            // 그리퍼 동기화 토글 버튼
            if (GUILayout.Button("그리퍼 동기화 토글"))
            {
                ToggleGripperSync();
            }
        }
        
        GUILayout.EndArea();
    }
    
    /// <summary>
    /// 시스템 종료 시 정리 작업 메서드
    /// 
    /// Unity에서 GameObject가 파괴될 때 자동으로 호출됩니다.
    /// 시스템 종료 로그를 출력하여 정상적인 종료를 확인할 수 있습니다.
    /// </summary>
    void OnDestroy()
    {
        if (enableDebugLog)
            Debug.Log("WX250s Unity 팔로워 시스템 종료");
    }
}

/// <summary>
/// 시스템 상태 정보를 담는 구조체
/// 
/// 팔로워 시스템의 현재 상태를 나타내는 데이터를 담는 구조체입니다.
/// GetSystemStatus() 메서드를 통해 시스템의 모든 주요 상태 정보를
/// 한 번에 확인할 수 있도록 합니다.
/// </summary>
[System.Serializable]
public class SystemStatus
{
    [Tooltip("ROS2 연결 상태 (true: 연결됨, false: 연결 안됨)")]
    public bool isConnected;
    
    [Tooltip("시스템 초기화 완료 여부 (true: 초기화 완료, false: 초기화 미완료)")]
    public bool isInitialized;
    
    [Tooltip("수신된 메시지 수 (ROS2에서 받은 총 메시지 개수)")]
    public int receivedMessageCount;
    
    [Tooltip("마지막 업데이트 시간 (Time.time 기준)")]
    public float lastUpdateTime;
    
    [Tooltip("연결 지속 시간 (초, 연결된 상태에서 경과된 시간)")]
    public float connectionUptime;
    
    [Tooltip("현재 그리퍼 값 (0~1, 0: 열림, 1: 닫힘)")]
    public float currentGripperValue;
}