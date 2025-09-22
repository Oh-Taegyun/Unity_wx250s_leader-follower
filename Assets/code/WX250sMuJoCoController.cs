/*
 * WX250sMuJoCoController.cs
 * wx250s957 로봇팔의 MuJoCo Unity 제어를 담당하는 메인 컨트롤러 클래스
 * 
 * 이 스크립트는 ROS2에서 받은 관절 명령을 MuJoCo 시뮬레이션에 적용하고 상태를 관리합니다.
 * ROS2Bridge와 UnityJointMapper와 함께 작동하여 완전한 ROS2-Unity 통합을 제공합니다.
 * 
 * 주요 기능:
 * - ROS2 관절 명령 수신 및 처리
 * - MuJoCo 시뮬레이션에 관절 명령 적용
 * - 부드러운 움직임을 위한 보간 처리
 * - 그리퍼 특별 처리 (ROS2 1개 → Unity 2개)
 * - 실시간 관절 상태 모니터링 및 피드백
 * - 런타임 디버그 정보 표시 및 제어
 * 
 * 사용 예시:
 * - ROS2Bridge → WX250sMuJoCoController → MuJoCo 시뮬레이션
 * - UnityJointMapper를 통한 관절 데이터 변환
 * - 실시간 로봇 팔 제어 및 시각화
 */

using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Mujoco;  // Unity MuJoCo 패키지

/// <summary>
/// wx250s957 로봇팔의 MuJoCo Unity 제어 컨트롤러
/// 
/// 이 클래스는 ROS2에서 받은 관절 명령을 MuJoCo 시뮬레이션에 적용하고 상태를 관리합니다.
/// ROS2Bridge와 UnityJointMapper와 함께 작동하여 완전한 ROS2-Unity 통합을 제공합니다.
/// 
/// 주요 역할:
/// - ROS2 관절 명령 수신 및 처리
/// - MuJoCo 시뮬레이션에 관절 명령 적용
/// - 부드러운 움직임을 위한 보간 처리
/// - 그리퍼 특별 처리 (ROS2 1개 → Unity 2개)
/// - 실시간 관절 상태 모니터링 및 피드백
/// </summary>
public class WX250sMuJoCoController : MonoBehaviour
{
    [Header("MuJoCo 모델 설정")]
    [Tooltip("MuJoCo 시뮬레이션 모델의 루트 Transform (모든 관절과 액추에이터의 부모)")]
    public Transform modelRoot;
    
    [Header("관절 설정 - Unity MuJoCo 구조")]
    [Tooltip("Unity MuJoCo에서 사용하는 관절 이름 배열 (8개, 그리퍼는 left_finger, right_finger로 분할)")]
    public string[] jointNames = {
        "waist",        // 인덱스 0: 허리 회전
        "shoulder",     // 인덱스 1: 어깨 관절
        "elbow",        // 인덱스 2: 팔꿈치 관절
        "forearm_roll", // 인덱스 3: 팔뚝 회전
        "wrist_angle",  // 인덱스 4: 손목 각도
        "wrist_rotate", // 인덱스 5: 손목 회전
        "left_finger",  // 인덱스 6: 왼쪽 손가락 (그리퍼)
        "right_finger"  // 인덱스 7: 오른쪽 손가락 (그리퍼)
    };
    
    [Header("제어 설정")]
    [Tooltip("제어 업데이트 주기 (Hz, 높을수록 정밀하지만 CPU 부하 증가)")]
    public float updateRate = 100f;
    
    [Header("그리퍼 설정")]
    [Tooltip("그리퍼 손가락 동기화 활성화 (left_finger = right_finger)")]
    public bool enableGripperSync = true;

    [Tooltip("그리퍼 동기화 허용 오차 (이 값보다 작은 차이는 무시)")]
    public float gripperSyncTolerance = 0.001f;
    
    [Header("디버그 설정")]
    [Tooltip("디버그 정보 표시 여부 (런타임 GUI 및 콘솔 로그)")]
    public bool showDebugInfo = true;
    
    [Tooltip("부드러운 움직임 활성화 (급격한 움직임 방지)")]
    public bool enableSmoothing = true;
    
    [Tooltip("부드러운 움직임 보간 계수 (0~1, 높을수록 빠른 움직임)")]
    public float smoothingFactor = 0.5f;
    
    // === 내부 상태 변수들 ===
    
    // 관절 상태 배열들 (8개 관절)
    private float[] currentPositions;    // 현재 관절 위치 (라디안)
    private float[] targetPositions;     // 목표 관절 위치 (라디안)
    private float[] currentVelocities;   // 현재 관절 속도 (라디안/초)
    private float[] targetVelocities;    // 목표 관절 속도 (라디안/초)
    
    // 제어 타이밍 변수들
    private float updateInterval;        // 업데이트 간격 (초, 1/updateRate)
    private float lastUpdateTime;        // 마지막 업데이트 시간 (Time.time)
    
    // 관절 매핑 (ROS2 관절명 → Unity MuJoCo 인덱스)
    private Dictionary<string, int> jointMapping;
    
    // 그리퍼 상태 변수들
    private float currentGripperValue = 0f;  // 현재 그리퍼 값 (0: 열림, 1: 닫힘)
    private float targetGripperValue = 0f;   // 목표 그리퍼 값 (0: 열림, 1: 닫힘)
    
    // ROS2 → Unity 관절 매핑 (값, 방향)
    private static readonly (int index, bool invert)[] ROS2_TO_UNITY_MAPPING = {
        (2, false), // 0: waist (방향 유지)
        (1, false),  // 1: shoulder (방향 반전)
        (3, false),  // 2: elbow (방향 반전)  
        (4, true),  // 3: forearm_roll (방향 반전)
        (5, false), // 4: wrist_angle (방향 유지)
        (0, false), // 5: wrist_rotate (방향 유지)
        (6, false)  // 6: gripper (방향 유지)
    };

    /// <summary>
    /// Unity Start 메서드
    /// 스크립트가 활성화될 때 한 번 호출되어 모든 초기화 작업을 수행함
    /// </summary>
    void Start()
    {
        InitializeMuJoCo();      // MuJoCo 모델 초기화
        InitializeJointMapping(); // 관절 매핑 초기화
        InitializeArrays();      // 관절 상태 배열 초기화
        
        // 제어 타이밍 설정
        updateInterval = 1f / updateRate;  // 업데이트 간격 계산 (초)
        lastUpdateTime = Time.time;        // 초기 시간 설정
    }
    
    /// <summary>
    /// MuJoCo 모델 초기화 메서드
    /// 
    /// MuJoCo 시뮬레이션 모델의 루트 Transform을 확인하고
    /// 액추에이터들을 찾아서 모델이 올바르게 로드되었는지 확인합니다.
    /// </summary>
    void InitializeMuJoCo()
    {
        if (modelRoot == null)
        {
            Debug.LogError("MuJoCo 모델 루트가 설정되지 않았습니다! Inspector에서 modelRoot를 설정해주세요.");
            return;
        }
        
        // MuJoCo 모델 루트에서 액추에이터들 찾기
        // 각 관절에 붙어있는 컴포넌트임
        var actuators = modelRoot.GetComponentsInChildren<MjActuator>(true);
        
        if (showDebugInfo)
            Debug.Log($"MuJoCo 모델 초기화 완료: {jointNames.Length}개 관절, {actuators.Length}개 액추에이터");
    }
    
    /// <summary>
    /// ROS2와 Unity MuJoCo 관절 매핑 초기화 메서드
    /// 
    /// ROS2 관절 이름을 Unity MuJoCo 관절 인덱스로 매핑하는 딕셔너리를 생성합니다.
    /// 이 매핑은 ROS2 메시지를 Unity MuJoCo 형식으로 변환할 때 사용됩니다.
    /// </summary>
    void InitializeJointMapping()
    {
        jointMapping = new Dictionary<string, int>();

        for (int i = 0; i < jointNames.Length; i++)
        {
            jointMapping[jointNames[i]] = i;
        }
    }
    
    /// <summary>
    /// 관절 상태 배열들 초기화 메서드
    /// 
    /// 모든 관절의 현재/목표 위치와 속도를 저장할 배열들을 생성하고
    /// 초기값을 0으로 설정합니다.
    /// </summary>
    // void InitializeArrays()
    // {
    //     int jointCount = jointNames.Length;  // 8개 관절
    //     currentPositions = new float[jointCount];   // 현재 관절 위치
    //     targetPositions = new float[jointCount];    // 목표 관절 위치
    //     currentVelocities = new float[jointCount];  // 현재 관절 속도
    //     targetVelocities = new float[jointCount];   // 목표 관절 속도
        
    //     // 모든 관절을 0 위치로 초기화 (중립 상태)
    //     for (int i = 0; i < jointCount; i++)
    //     {
    //         currentPositions[i] = 0f;   // 현재 위치: 0 (라디안)
    //         targetPositions[i] = 0f;    // 목표 위치: 0 (라디안)
    //         currentVelocities[i] = 0f;  // 현재 속도: 0 (라디안/초)
    //         targetVelocities[i] = 0f;   // 목표 속도: 0 (라디안/초)
    //     }
    // }
    
    /// <summary>
    /// 관절 상태 배열들 초기화 메서드 (새로운 초기 포즈)
    /// 
    /// 모든 관절의 현재/목표 위치와 속도를 저장할 배열들을 생성하고
    /// 지정된 초기 포즈로 설정합니다.
    /// </summary>
    void InitializeArrays()
    {
        int jointCount = jointNames.Length;  // 8개 관절
        currentPositions = new float[jointCount];   // 현재 관절 위치
        targetPositions = new float[jointCount];    // 목표 관절 위치
        currentVelocities = new float[jointCount];  // 현재 관절 속도
        targetVelocities = new float[jointCount];   // 목표 관절 속도
        
        // ROS2 순서의 초기 포즈 (ROS2_TO_UNITY_MAPPING 적용 전)
        float[] ros2InitialPositions = {
            1.526f,    // 0: elbow
            -1.698f,    // 1: shoulder
            -0.101f,     // 2: forearm_roll
            0.270f,    // 3: wrist_angle
            0.089f,     // 4: wrist_rotate
            -0.006f,     // 5: waist
            0.000f          // 6: gripper
        };
        
        // Unity MuJoCo 순서로 변환하여 초기 포즈 설정
        for (int i = 0; i < jointCount; i++)
        {
            if (i < 6) // 기본 관절 6개 (waist ~ wrist_rotate)
            {
                // ROS2_TO_UNITY_MAPPING을 사용하여 ROS2 순서를 Unity 순서로 변환
                if (i < ROS2_TO_UNITY_MAPPING.Length)
                {
                    var (unityIndex, invert) = ROS2_TO_UNITY_MAPPING[i];
                    if (unityIndex < jointCount)
                    {
                        float value = ros2InitialPositions[i];
                        // if (invert)
                        // {
                        //     value = -value; // 방향 반전
                        // }
                        currentPositions[unityIndex] = value;   // 현재 위치: 변환된 초기값
                        targetPositions[unityIndex] = value;    // 목표 위치: 변환된 초기값
                    }
                }
            }
            else // 그리퍼 관절들 (left_finger, right_finger)
            {
                currentPositions[i] = 0f;   // 그리퍼는 열림 상태
                targetPositions[i] = 0f;    // 그리퍼는 열림 상태
            }
            
            currentVelocities[i] = 0f;  // 현재 속도: 0 (라디안/초)
            targetVelocities[i] = 0f;   // 목표 속도: 0 (라디안/초)
        }
    }

    // /// <summary>
    // /// ROS2에서 받은 관절 상태 메시지를 처리하는 메서드
    // /// 
    // /// ROS2Bridge에서 호출되는 메인 인터페이스입니다.
    // /// ROS2 JointState 메시지를 받아서 Unity MuJoCo 관절 형식으로 변환하고
    // /// 목표 위치로 설정합니다.
    // /// </summary>
    // /// <param name="jointState">ROS2 JointState 메시지 (관절 이름, 위치, 속도 포함)</param>
    // public void UpdateJointStates(JointStateMessage jointState)
    // {
    //     if (jointState == null || jointState.name == null || jointState.position == null)
    //     {
    //         Debug.LogWarning("잘못된 관절 상태 메시지");
    //         return;
    //     }
        
    //     // ROS2 관절 데이터를 Unity MuJoCo 관절로 매핑
    //     MapROS2ToUnityMuJoCo(jointState);
        
    //     if (showDebugInfo)
    //     {
    //         Debug.Log($"관절 상태 업데이트: {string.Join(", ", targetPositions)}");
    //     }
    // }
    
    /// <summary>
    /// ROS2에서 받은 궤적 명령을 처리하는 메서드
    /// 
    /// ROS2Bridge에서 호출되는 궤적 실행 인터페이스입니다.
    /// ROS2 JointTrajectory 메시지를 받아서 첫 번째 포인트를 즉시 실행합니다.
    /// 실시간 제어를 위해 궤적의 첫 번째 포인트만 사용합니다.
    /// </summary>
    /// <param name="trajectory">ROS2 JointTrajectory 메시지 (궤적 포인트들 포함)</param>
    public void ExecuteTrajectory(JointTrajectoryMessage trajectory)
    {
        if (trajectory == null || trajectory.points == null || trajectory.points.Length == 0)
        {
            Debug.LogWarning("잘못된 궤적 메시지");
            return;
        }
        
        // 첫 번째 궤적 포인트를 즉시 실행 (실시간 제어)
        var firstPoint = trajectory.points[0];
        
        if (firstPoint.positions != null && firstPoint.positions.Length > 0)
        {
            // ROS2 → Unity 관절 순서 매핑
            for (int i = 0; i < Mathf.Min(firstPoint.positions.Length, targetPositions.Length); i++)
            {
                if (i < ROS2_TO_UNITY_MAPPING.Length)
                {
                    var (unityIndex, invert) = ROS2_TO_UNITY_MAPPING[i];
                    if (unityIndex < targetPositions.Length)
                    {
                        float value = firstPoint.positions[i];
                        // if (invert)
                        // {
                        //     value = -value; // 방향 반전
                        // }
                        targetPositions[unityIndex] = value;
                    }
                }
            }
            
            if (showDebugInfo)
            {
                Debug.Log($"궤적 실행: {string.Join(", ", firstPoint.positions)}");
            }
        }

        // 그리퍼 특별 처리 (ROS2 6번 → Unity 6,7번)
        if (firstPoint.positions.Length > 6)
        {
            float rosGripperValue = firstPoint.positions[6];
            
            // 그리퍼 값을 이진화 처리
            if (rosGripperValue < 0)
            {
                SetGripperValue(1f); // 완전 열림
            }
            else
            {
                SetGripperValue(0f); // 완전 닫힘
            }
        }
    }
    
    // /// <summary>
    // /// ROS2 관절 데이터를 Unity MuJoCo 형식으로 매핑하는 메서드
    // /// 
    // /// ROS2의 7개 관절을 Unity MuJoCo의 8개 관절로 변환합니다.
    // /// 그리퍼는 ROS2에서 1개이지만 Unity에서는 left_finger와 right_finger로 분할됩니다.
    // /// 
    // /// 매핑 과정:
    // /// 1. 기본 관절 6개 (waist ~ wrist_rotate) 1:1 매핑
    // /// 2. 그리퍼 관절 1개 (ROS2) → 2개 (Unity) 분할 매핑
    // /// 3. 그리퍼 동기화 (left_finger = right_finger)
    // /// </summary>
    // /// <param name="jointState">ROS2 관절 상태 메시지 (7개 관절 데이터)</param>
    // void MapROS2ToUnityMuJoCo(JointStateMessage jointState)
    // {
    //     // === 기본 관절 매핑 (waist ~ wrist_rotate, 6개) ===
    //     // ROS2와 Unity MuJoCo에서 동일한 순서로 1:1 매핑
    //     for (int i = 0; i < 6; i++)
    //     {
    //         if (i < jointState.position.Length)
    //         {
    //             targetPositions[i] = jointState.position[i];  // 목표 위치 설정
    //             if (jointState.velocity != null && i < jointState.velocity.Length)
    //             {
    //                 targetVelocities[i] = jointState.velocity[i];  // 목표 속도 설정
    //             }
    //         }
    //     }
        
    //     // === 그리퍼 특별 처리 (ROS2 1개 → Unity MuJoCo 2개) ===
    //     if (jointState.position.Length > 6)
    //     {
    //         targetGripperValue = jointState.position[6]; // ROS2 gripper 값 (인덱스 6)
            
    //         if (enableGripperSync)
    //         {
    //             // left_finger와 right_finger를 동일한 값으로 설정
    //             targetPositions[6] = targetGripperValue; // left_finger (인덱스 6)
    //             targetPositions[7] = targetGripperValue; // right_finger (인덱스 7)
    //         }
    //     }
    // }
    
    /// <summary>
    /// Unity Update 루프 - 고정 주기로 MuJoCo 제어 실행
    /// 
    /// Unity의 Update 메서드에서 고정된 주기(기본 100Hz)로 MuJoCo 제어를 실행합니다.
    /// 이렇게 하면 프레임률에 관계없이 일정한 제어 주기를 유지할 수 있습니다.
    /// </summary>
    void Update()
    {
        if (modelRoot == null) return;
        
        // 고정된 업데이트 주기로 제어 (100Hz 기본)
        if (Time.time - lastUpdateTime >= updateInterval)
        {
            UpdateMuJoCoControl();  // MuJoCo 제어 메인 루프 실행
            lastUpdateTime = Time.time;  // 마지막 업데이트 시간 갱신
        }
    }
    
    /// <summary>
    /// MuJoCo 제어 메인 루프 메서드
    /// 
    /// 고정 주기로 실행되는 MuJoCo 제어의 핵심 로직입니다.
    /// 다음 순서로 실행됩니다:
    /// 1. 부드러운 움직임을 위한 보간
    /// 2. 그리퍼 동기화 확인 및 보정
    /// 3. MuJoCo 모델에 관절 명령 적용
    /// 4. 현재 관절 상태 업데이트 (피드백)
    /// </summary>
    void UpdateMuJoCoControl()
    {
        // 1) 부드러운 움직임을 위한 보간
        if (enableSmoothing)
        {
            SmoothJointPositions();
        }
        
        // 2) 그리퍼 동기화 확인 및 보정
        if (enableGripperSync)
        {
            SyncGripperFingers();
        }
        
        // 3) MuJoCo 모델에 관절 명령 적용
        ApplyJointCommands();
        
        // 4) 현재 관절 상태 업데이트 (피드백)
        UpdateCurrentStates();
    }
    
    /// <summary>
    /// 관절 위치와 속도를 부드럽게 보간하는 메서드
    /// 
    /// 급격한 움직임을 방지하고 자연스러운 애니메이션을 구현합니다.
    /// 선형 보간(Lerp)을 사용하여 현재 위치에서 목표 위치로 부드럽게 이동합니다.
    /// smoothingFactor 값에 따라 움직임의 속도가 결정됩니다.
    /// </summary>
    void SmoothJointPositions()
    {
        for (int i = 0; i < currentPositions.Length; i++)
        {
            // 선형 보간으로 부드러운 움직임 구현
            currentPositions[i] = Mathf.Lerp(currentPositions[i], targetPositions[i], smoothingFactor);
            currentVelocities[i] = Mathf.Lerp(currentVelocities[i], targetVelocities[i], smoothingFactor);
        }
        
        // 그리퍼 값도 부드럽게 보간
        currentGripperValue = Mathf.Lerp(currentGripperValue, targetGripperValue, smoothingFactor);
    }
    
    /// <summary>
    /// 그리퍼 손가락 동기화 확인 및 보정 메서드
    /// 
    /// left_finger와 right_finger가 허용 오차 내에서 동일한 값인지 확인하고,
    /// 동기화가 깨진 경우 평균값으로 보정합니다.
    /// 이렇게 하면 그리퍼가 대칭적으로 움직이게 됩니다.
    /// </summary>
    void SyncGripperFingers()
    {
        // left_finger와 right_finger 위치 확인
        float leftPos = currentPositions[6];   // left_finger (인덱스 6)
        float rightPos = currentPositions[7];  // right_finger (인덱스 7)
        
        // 허용 오차를 초과하는 경우 동기화 보정
        if (Mathf.Abs(leftPos - rightPos) > gripperSyncTolerance)
        {
            // 동기화되지 않은 경우 평균값으로 설정
            float avgPos = (leftPos + rightPos) / 2f;
            currentPositions[6] = avgPos;  // left_finger
            currentPositions[7] = avgPos;  // right_finger
            
            if (showDebugInfo)
                Debug.LogWarning($"그리퍼 동기화 보정: {leftPos:F3} → {avgPos:F3}");
        }
    }
    
    /// <summary>
    /// MuJoCo 모델에 관절 명령을 적용하는 메서드
    /// 
    /// 현재 관절 위치와 속도를 MuJoCo 시뮬레이션에 전달합니다.
    /// 각 관절에 대응하는 MjActuator를 찾아서 Control 값을 설정합니다.
    /// 그리퍼는 특별히 "gripper" 액추에이터에 매핑됩니다.
    /// </summary>
    void ApplyJointCommands()
    {
        if (modelRoot == null) return;
        
        // 모든 액추에이터 찾기
        var actuators = modelRoot.GetComponentsInChildren<MjActuator>(true);
        
        for (int i = 0; i < jointNames.Length; i++)
        {
            try
            {
                // 해당 관절 이름과 일치하는 액추에이터 찾기
                string targetJointName = jointNames[i];
                
                // 그리퍼 특별 처리: gripper actuator는 left_finger에 매핑
                if (targetJointName == "left_finger")
                {
                    targetJointName = "gripper";
                }
                
                var actuator = System.Array.Find(actuators, a => a.name == targetJointName);
                
                if (actuator != null)
                {
                    // 액추에이터에 제어 값 설정 (라디안 단위)
                    actuator.Control = currentPositions[i];
                }
                else
                {
                    if (showDebugInfo)
                        Debug.LogWarning($"액추에이터를 찾을 수 없음: {targetJointName}");
                }
            }
            catch (Exception ex)
            {
                Debug.LogError($"관절 {jointNames[i]} 제어 오류: {ex.Message}");
            }
        }
    }
    
    /// <summary>
    /// MuJoCo 모델에서 현재 관절 상태를 읽어오는 메서드
    /// 
    /// 시뮬레이션 결과를 피드백으로 받아와 현재 상태를 업데이트합니다.
    /// 각 관절의 MjActuator에서 Control 값을 읽어와서 현재 위치로 설정합니다.
    /// 이는 제어 루프의 피드백 역할을 합니다.
    /// </summary>
    void UpdateCurrentStates()
    {
        if (modelRoot == null) return;
        
        // 모든 액추에이터와 관절 찾기
        var actuators = modelRoot.GetComponentsInChildren<MjActuator>(true);
        
        for (int i = 0; i < jointNames.Length; i++)
        {
            try
            {
                // 해당 관절 이름과 일치하는 액추에이터 찾기
                string targetJointName = jointNames[i];
                
                // 그리퍼 특별 처리: gripper actuator는 left_finger에 매핑
                if (targetJointName == "left_finger")
                {
                    targetJointName = "gripper";
                }
                
                var actuator = System.Array.Find(actuators, a => a.name == targetJointName);
                
                if (actuator != null)
                {
                    // 액추에이터에서 현재 제어 값 읽기 (라디안 단위)
                    currentPositions[i] = actuator.Control;
                    
                    // 속도는 현재 구현에서는 0으로 설정 (필요시 확장 가능)
                    currentVelocities[i] = 0f;
                }
            }
            catch (Exception ex)
            {
                Debug.LogError($"관절 {jointNames[i]} 상태 읽기 오류: {ex.Message}");
            }
        }
    }
    
    // === 공개 API 메서드들 ===
    // 다른 스크립트에서 WX250sMuJoCoController의 상태를 확인하거나 제어할 수 있는 메서드들
    
    /// <summary>
    /// 현재 관절 위치 배열을 반환하는 메서드
    /// 
    /// 현재 시뮬레이션에서의 관절 위치를 반환합니다.
    /// 배열의 복사본을 반환하므로 원본 데이터는 보호됩니다.
    /// </summary>
    /// <returns>현재 관절 위치 배열의 복사본 (8개 관절, 라디안 단위)</returns>
    public float[] GetCurrentPositions()
    {
        return (float[])currentPositions.Clone();
    }
    
    /// <summary>
    /// 목표 관절 위치 배열을 반환하는 메서드
    /// 
    /// 현재 설정된 목표 관절 위치를 반환합니다.
    /// 배열의 복사본을 반환하므로 원본 데이터는 보호됩니다.
    /// </summary>
    /// <returns>목표 관절 위치 배열의 복사본 (8개 관절, 라디안 단위)</returns>
    public float[] GetTargetPositions()
    {
        return (float[])targetPositions.Clone();
    }
    
    /// <summary>
    /// 인덱스로 관절 위치를 설정하는 메서드
    /// 
    /// 관절 인덱스를 사용하여 목표 위치를 설정합니다.
    /// 인덱스 범위: 0~7 (waist, shoulder, elbow, forearm_roll, wrist_angle, wrist_rotate, left_finger, right_finger)
    /// </summary>
    /// <param name="jointIndex">관절 인덱스 (0~7)</param>
    /// <param name="position">목표 위치 (라디안 단위)</param>
    public void SetJointPosition(int jointIndex, float position)
    {
        if (jointIndex >= 0 && jointIndex < targetPositions.Length)
        {
            targetPositions[jointIndex] = position;
        }
    }
    
    /// <summary>
    /// 관절 이름으로 관절 위치를 설정하는 메서드
    /// 
    /// 관절 이름을 사용하여 목표 위치를 설정합니다.
    /// 관절 이름: "waist", "shoulder", "elbow", "forearm_roll", "wrist_angle", "wrist_rotate", "left_finger", "right_finger"
    /// </summary>
    /// <param name="jointName">관절 이름</param>
    /// <param name="position">목표 위치 (라디안 단위)</param>
    public void SetJointPosition(string jointName, float position)
    {
        for (int i = 0; i < jointNames.Length; i++)
        {
            if (jointNames[i] == jointName)
            {
                targetPositions[i] = position;
                break;
            }
        }
    }
    
    // === 그리퍼 제어 메서드들 ===
    // 그리퍼 관절의 특별한 처리를 위한 메서드들
    
    /// <summary>
    /// 그리퍼 값을 설정하는 메서드
    /// 
    /// 그리퍼의 열림/닫힘 상태를 설정합니다.
    /// 0: 완전히 열림, 1: 완전히 닫힘
    /// left_finger와 right_finger 모두 동일한 값으로 설정됩니다.
    /// </summary>
    /// <param name="value">그리퍼 값 (0~1, 0: 열림, 1: 닫힘)</param>
    public void SetGripperValue(float value)
    {
        // 0~1 범위를 실제 물리적 범위로 변환
        // 0 → -0.008 (완전 열림)
        // 1 → 0.017 (완전 닫힘)
        float physicalValue = Mathf.Lerp(0, 1, value);
        targetGripperValue = physicalValue;
        targetPositions[6] = targetGripperValue; // left_finger (인덱스 6)
        targetPositions[7] = targetGripperValue; // right_finger (인덱스 7)
    }
    
    /// <summary>
    /// 현재 그리퍼 값을 반환하는 메서드
    /// 
    /// 현재 그리퍼의 상태를 반환합니다.
    /// 0: 완전히 열림, 1: 완전히 닫힘
    /// </summary>
    /// <returns>현재 그리퍼 값 (0~1)</returns>
    public float GetGripperValue()
    {
        return currentGripperValue;
    }
    
    /// <summary>
    /// 그리퍼 동기화를 설정하는 메서드
    /// 
    /// left_finger와 right_finger의 동기화를 활성화/비활성화합니다.
    /// 동기화가 활성화되면 두 손가락이 항상 동일한 값을 가집니다.
    /// </summary>
    /// <param name="enable">동기화 활성화 여부 (true: 동기화, false: 독립)</param>
    public void SetGripperSync(bool enable)
    {
        enableGripperSync = enable;
    }
    
    /// <summary>
    /// Unity GUI를 통한 디버그 정보 표시 및 제어 메서드
    /// 
    /// 런타임에서 관절 상태를 모니터링하고 그리퍼를 제어할 수 있습니다.
    /// 화면 좌측 상단에 제어 정보와 관절 상태를 표시하며,
    /// 그리퍼 제어 버튼을 제공합니다.
    /// 
    /// 표시 정보:
    /// - 제어 설정 (업데이트 주기, 부드러운 움직임, 그리퍼 동기화)
    /// - 관절 상태 (현재 위치 → 목표 위치)
    /// - 그리퍼 상태 및 제어 버튼
    /// </summary>
    void OnGUI()
    {
        if (!showDebugInfo) return;
        
        // GUI 영역 설정 (화면 좌측 상단)
        GUILayout.BeginArea(new Rect(10, 10, 350, 550));
        GUILayout.Label("WX250s Unity MuJoCo Controller", GUI.skin.box);
        
        // === 제어 설정 정보 표시 ===
        GUILayout.Label($"업데이트 주기: {updateRate}Hz");
        GUILayout.Label($"부드러운 움직임: {(enableSmoothing ? "ON" : "OFF")}");
        GUILayout.Label($"그리퍼 동기화: {(enableGripperSync ? "ON" : "OFF")}");
        
        GUILayout.Space(10);
        
        // === 관절 상태 표시 ===
        GUILayout.Label("관절 상태:");
        
        for (int i = 0; i < jointNames.Length; i++)
        {
            string jointInfo = $"{jointNames[i]}: {currentPositions[i]:F3} → {targetPositions[i]:F3}";
            if (i >= 6) // 그리퍼 관절들
            {
                jointInfo += " [GRIPPER]";
            }
            GUILayout.Label(jointInfo);
        }
        
        GUILayout.Space(10);
        
        // === 그리퍼 상태 표시 ===
        GUILayout.Label($"그리퍼 값: {currentGripperValue:F3} → {targetGripperValue:F3} (0: 열림, 1: 닫힘)");
        
        GUILayout.Space(10);
        
        // === 그리퍼 제어 버튼들 ===
        if (GUILayout.Button("그리퍼 닫기"))
        {
            SetGripperValue(0f);
        }
        
        if (GUILayout.Button("그리퍼 열기"))
        {
            SetGripperValue(1f);
        }
        
        GUILayout.Space(10);
        
        // === 씬 다시 불러오기 버튼 ===
        if (GUILayout.Button("씬 다시 불러오기"))
        {
            UnityEngine.SceneManagement.SceneManager.LoadScene(UnityEngine.SceneManagement.SceneManager.GetActiveScene().name);
        }
        
        GUILayout.EndArea();
    }
}