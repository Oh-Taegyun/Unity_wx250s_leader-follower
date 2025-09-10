/*
 * UnityJointMapper.cs
 * ROS2와 Unity MuJoCo 간의 관절 매핑을 담당하는 클래스
 * 
 * 이 스크립트는 ROS2와 Unity MuJoCo 시뮬레이션 간의 관절 데이터 변환을 담당합니다.
 * ROS2는 7개 관절을 사용하지만 Unity MuJoCo는 8개 관절(그리퍼 분할)을 사용하므로
 * 데이터 변환과 매핑이 필요합니다.
 * 
 * 주요 기능:
 * - ROS2 ↔ Unity MuJoCo 관절 데이터 변환
 * - 그리퍼 관절 특별 처리 (ROS2 1개 → Unity 2개)
 * - 관절 값 스케일링, 오프셋, 반전 처리
 * - 런타임 매핑 정보 표시 및 제어
 * 
 * 사용 예시:
 * - ROS2 JointTrajectory → Unity MuJoCo 관절 명령
 * - Unity MuJoCo 관절 상태 → ROS2 JointState
 */

using System;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// ROS2와 Unity MuJoCo 관절 간의 매핑 정보를 담는 구조체
/// 각 관절의 이름, 인덱스, 변환 파라미터 등을 포함하여 데이터 변환 규칙을 정의
/// </summary>
[System.Serializable]
public class JointMapping
{
    [Tooltip("ROS2에서 사용하는 관절 이름 (예: 'waist', 'shoulder', 'gripper')")]
    public string ros2JointName;
    
    [Tooltip("Unity MuJoCo에서 사용하는 관절 이름 (예: 'waist', 'shoulder', 'left_finger')")]
    public string unityJointName;
    
    [Tooltip("Unity MuJoCo 관절 배열의 인덱스 (0~7, 그리퍼는 6,7번 인덱스)")]
    public int unityIndex;
    
    [Tooltip("값 변환 시 적용할 스케일 팩터 (단위 변환, 범위 조정용)")]
    public float scaleFactor = 1f;
    
    [Tooltip("값 변환 시 적용할 오프셋 (기준점 조정용)")]
    public float offset = 0f;
    
    [Tooltip("값을 반전시킬지 여부 (방향이 반대인 관절용)")]
    public bool invert = false;
    
    [Tooltip("그리퍼 관절인지 여부 (ROS2 1개 → Unity 2개 매핑용)")]
    public bool isGripperJoint = false;
}

/// <summary>
/// ROS2와 Unity MuJoCo 간의 관절 매핑을 관리하는 메인 클래스
/// 
/// 이 클래스는 ROS2와 Unity MuJoCo 시뮬레이션 간의 관절 데이터 변환을 담당합니다.
/// 주요 역할:
/// - ROS2 7개 관절 ↔ Unity MuJoCo 8개 관절 변환
/// - 그리퍼 관절 특별 처리 (ROS2 1개 → Unity 2개)
/// - 관절 값 스케일링, 오프셋, 반전 처리
/// - 런타임 매핑 정보 표시 및 제어
/// </summary>
public class UnityJointMapper : MonoBehaviour
{
    [Header("관절 매핑 설정")]
    [Tooltip("ROS2와 Unity MuJoCo 관절 간의 매핑 정보 배열 (Inspector에서 설정 가능)")]
    public JointMapping[] jointMappings;
    
    [Header("그리퍼 설정")]
    [Tooltip("그리퍼 매핑 활성화 여부 (ROS2 gripper → Unity left_finger, right_finger)")]
    public bool enableGripperMapping = true;
    
    [Tooltip("그리퍼 값 변환 시 적용할 스케일 팩터 (단위 변환용)")]
    public float gripperScaleFactor = 1f;
    
    [Tooltip("그리퍼 값 변환 시 적용할 오프셋 (기준점 조정용)")]
    public float gripperOffset = 0f;
    
    [Tooltip("그리퍼 손가락 동기화 여부 (left_finger = right_finger로 동일하게 설정)")]
    public bool syncGripperFingers = true;
    
    [Header("디버그 설정")]
    [Tooltip("매핑 정보 GUI 표시 여부 (런타임에서 매핑 상태 확인 가능)")]
    public bool showMappingInfo = true;
    
    // === 내부 상태 변수들 ===
    private Dictionary<string, JointMapping> mappingDict;           // ROS2 관절명 -> 매핑 정보 (빠른 검색용)
    private Dictionary<string, List<JointMapping>> gripperMappings; // 그리퍼 관절별 매핑 리스트 (동기화용)
    
    /// <summary>
    /// Unity Start 메서드
    /// 스크립트가 활성화될 때 한 번 호출되어 매핑 시스템을 초기화함
    /// </summary>
    void Start()
    {
        InitializeMapping();
    }
    
    /// <summary>
    /// 관절 매핑 시스템 초기화 메서드
    /// 
    /// 이 메서드는 다음과 같은 작업을 수행합니다:
    /// 1. 매핑 딕셔너리 초기화 (빠른 검색을 위한 해시테이블)
    /// 2. 기본 매핑 생성 (Inspector에서 설정되지 않은 경우)
    /// 3. 그리퍼 매핑 별도 관리 (동기화를 위한 리스트)
    /// 4. 초기화 완료 로그 출력
    /// </summary>
    void InitializeMapping()
    {
        // 매핑 딕셔너리 초기화
        mappingDict = new Dictionary<string, JointMapping>();           // ROS2 관절명 -> 매핑 정보
        gripperMappings = new Dictionary<string, List<JointMapping>>(); // 그리퍼 관절별 매핑 리스트
        
        // 기본 매핑 설정 (Inspector에서 설정되지 않은 경우)
        if (jointMappings == null || jointMappings.Length == 0)
        {
            CreateDefaultMapping();
        }
        
        // 매핑 딕셔너리 생성 (빠른 검색을 위한 해시테이블 구성)
        foreach (var mapping in jointMappings)
        {
            mappingDict[mapping.ros2JointName] = mapping;
            
            // 그리퍼 매핑 별도 관리 (left_finger, right_finger 동기화용)
            if (mapping.isGripperJoint)
            {
                if (!gripperMappings.ContainsKey(mapping.ros2JointName))
                {
                    gripperMappings[mapping.ros2JointName] = new List<JointMapping>();
                }
                gripperMappings[mapping.ros2JointName].Add(mapping);
            }
        }
        
        if (showMappingInfo)
        {
            Debug.Log($"Unity 관절 매핑 초기화 완료: {mappingDict.Count}개 매핑");
            Debug.Log($"그리퍼 매핑: {gripperMappings.Count}개");
        }
    }
    
    /// <summary>
    /// 기본 관절 매핑 생성 메서드
    /// 
    /// wx250s957 로봇팔의 표준 관절 매핑을 자동으로 설정합니다.
    /// ROS2는 7개 관절을 사용하지만 Unity MuJoCo는 8개 관절을 사용합니다.
    /// 그리퍼는 ROS2에서 1개이지만 Unity에서는 left_finger와 right_finger로 분할됩니다.
    /// </summary>
    void CreateDefaultMapping()
    {
        jointMappings = new JointMapping[]
        {
            // === 기본 관절들 (ROS2 ↔ Unity MuJoCo 1:1 매핑) ===
            // ROS2와 Unity MuJoCo에서 동일한 이름과 순서를 사용
            new JointMapping { ros2JointName = "waist", unityJointName = "waist", unityIndex = 0 },
            new JointMapping { ros2JointName = "shoulder", unityJointName = "shoulder", unityIndex = 1 },
            new JointMapping { ros2JointName = "elbow", unityJointName = "elbow", unityIndex = 2 },
            new JointMapping { ros2JointName = "forearm_roll", unityJointName = "forearm_roll", unityIndex = 3 },
            new JointMapping { ros2JointName = "wrist_angle", unityJointName = "wrist_angle", unityIndex = 4 },
            new JointMapping { ros2JointName = "wrist_rotate", unityJointName = "wrist_rotate", unityIndex = 5 },
            
            // === 그리퍼 관절들 (ROS2 1개 → Unity MuJoCo 2개 매핑) ===
            // ROS2의 "gripper" → Unity의 "left_finger"와 "right_finger"로 분할
            // 두 손가락은 동일한 값을 가지며 동기화됩니다
            new JointMapping { 
                ros2JointName = "gripper", 
                unityJointName = "left_finger", 
                unityIndex = 6, 
                isGripperJoint = true 
            },
            new JointMapping { 
                ros2JointName = "gripper", 
                unityJointName = "right_finger", 
                unityIndex = 7, 
                isGripperJoint = true 
            }
        };
    }
    
    /// <summary>
    /// ROS2 관절 데이터를 Unity MuJoCo 형식으로 변환하는 메서드
    /// 
    /// ROS2의 7개 관절을 Unity MuJoCo의 8개 관절로 변환합니다.
    /// 그리퍼는 ROS2에서 1개이지만 Unity에서는 left_finger와 right_finger로 분할됩니다.
    /// 
    /// 변환 과정:
    /// 1. ROS2 관절 이름으로 매핑 정보 검색
    /// 2. 스케일, 오프셋, 반전 적용
    /// 3. 그리퍼 관절 특별 처리 (1개 → 2개)
    /// 4. 그리퍼 손가락 동기화 (left_finger = right_finger)
    /// </summary>
    /// <param name="ros2Positions">ROS2 관절 위치 배열 (7개)</param>
    /// <param name="ros2JointNames">ROS2 관절 이름 배열 (7개)</param>
    /// <returns>Unity MuJoCo 관절 위치 배열 (8개)</returns>
    public float[] MapROS2ToUnityMuJoCo(float[] ros2Positions, string[] ros2JointNames)
    {
        if (ros2Positions == null || ros2JointNames == null)
        {
            Debug.LogWarning("⚠️ ROS2 관절 데이터가 null입니다.");
            return new float[8]; // Unity MuJoCo 관절 수만큼 반환
        }
        
        float[] unityPositions = new float[8]; // Unity MuJoCo 관절 수 (0~7)
        
        // ROS2 관절 데이터를 Unity MuJoCo 형식으로 변환
        for (int i = 0; i < ros2JointNames.Length && i < ros2Positions.Length; i++)
        {
            string ros2JointName = ros2JointNames[i];
            float ros2Position = ros2Positions[i];
            
            if (mappingDict.ContainsKey(ros2JointName))
            {
                var mapping = mappingDict[ros2JointName];
                
                // 그리퍼 관절 특별 처리 (ROS2 1개 → Unity 2개)
                if (mapping.isGripperJoint && enableGripperMapping)
                {
                    float mappedPosition = ApplyMapping(ros2Position, mapping);
                    unityPositions[mapping.unityIndex] = mappedPosition;
                }
                // 일반 관절 처리 (ROS2 1개 → Unity 1개)
                else if (!mapping.isGripperJoint)
                {
                    float mappedPosition = ApplyMapping(ros2Position, mapping);
                    unityPositions[mapping.unityIndex] = mappedPosition;
                }
            }
        }
        
        // 그리퍼 손가락 동기화 (left_finger = right_finger)
        if (enableGripperMapping && syncGripperFingers)
        {
            SyncGripperPositions(unityPositions);
        }
        
        return unityPositions;
    }
    
    /// <summary>
    /// 그리퍼 손가락 위치 동기화 메서드
    /// 
    /// left_finger와 right_finger를 동일한 값으로 설정합니다.
    /// 두 손가락의 평균값을 계산하여 양쪽 모두에 적용합니다.
    /// 이렇게 하면 그리퍼가 대칭적으로 움직이게 됩니다.
    /// </summary>
    /// <param name="unityPositions">Unity MuJoCo 관절 위치 배열 (8개)</param>
    void SyncGripperPositions(float[] unityPositions)
    {
        // left_finger와 right_finger를 동일한 값으로 동기화
        if (unityPositions.Length >= 8)
        {
            float leftFinger = unityPositions[6];   // left_finger (인덱스 6)
            float rightFinger = unityPositions[7];  // right_finger (인덱스 7)
            float avgValue = (leftFinger + rightFinger) / 2f;  // 평균값 계산
            
            unityPositions[6] = avgValue; // left_finger에 평균값 적용
            unityPositions[7] = avgValue; // right_finger에 평균값 적용
        }
    }
    
    /// <summary>
    /// Unity MuJoCo 관절 데이터를 ROS2 형식으로 변환하는 메서드
    /// 
    /// Unity MuJoCo의 8개 관절을 ROS2의 7개 관절로 변환합니다.
    /// 그리퍼는 Unity에서 left_finger와 right_finger 2개이지만 ROS2에서는 gripper 1개로 통합됩니다.
    /// 
    /// 변환 과정:
    /// 1. Unity 관절 값을 역변환 (반전, 오프셋, 스케일 해제)
    /// 2. 그리퍼 관절 통합 (Unity 2개 → ROS2 1개)
    /// 3. ROS2 표준 순서로 배열 구성
    /// </summary>
    /// <param name="unityPositions">Unity MuJoCo 관절 위치 배열 (8개)</param>
    /// <returns>ROS2 관절 위치 배열 (7개)</returns>
    public float[] MapUnityMuJoCoToROS2(float[] unityPositions)
    {
        if (unityPositions == null)
        {
            return new float[7]; // ROS2 관절 수만큼 반환
        }
        
        float[] ros2Positions = new float[7]; // ROS2 관절 수 (0~6)
        
        foreach (var mapping in jointMappings)
        {
            if (mapping.unityIndex < unityPositions.Length)
            {
                float unityValue = unityPositions[mapping.unityIndex];
                float ros2Value = ApplyInverseMapping(unityValue, mapping);
                
                // 그리퍼는 하나의 값으로 통합 (Unity 2개 → ROS2 1개)
                if (mapping.ros2JointName == "gripper")
                {
                    ros2Positions[6] = ros2Value;  // gripper는 인덱스 6
                }
                else
                {
                    // 일반 관절은 이름을 인덱스로 변환
                    int ros2Index = GetROS2JointIndex(mapping.ros2JointName);
                    if (ros2Index >= 0 && ros2Index < ros2Positions.Length)
                    {
                        ros2Positions[ros2Index] = ros2Value;
                    }
                }
            }
        }
        
        return ros2Positions;
    }
    
    /// <summary>
    /// ROS2 → Unity MuJoCo 값 변환 적용 메서드
    /// 
    /// 관절 값을 ROS2 형식에서 Unity MuJoCo 형식으로 변환합니다.
    /// 변환 순서: 스케일 → 오프셋 → 반전
    /// 
    /// 예시:
    /// - 스케일: 라디안 → 도 단위 변환 (scaleFactor = 180/π)
    /// - 오프셋: 기준점 조정 (offset = 90)
    /// - 반전: 방향 반대 (invert = true)
    /// </summary>
    /// <param name="inputValue">입력 값 (ROS2 형식)</param>
    /// <param name="mapping">매핑 정보 (변환 파라미터)</param>
    /// <returns>변환된 값 (Unity MuJoCo 형식)</returns>
    float ApplyMapping(float inputValue, JointMapping mapping)
    {
        float mappedValue = inputValue;
        
        // 1) 스케일 적용 (단위 변환 등)
        mappedValue *= mapping.scaleFactor;
        
        // 2) 오프셋 적용 (기준점 조정)
        mappedValue += mapping.offset;
        
        // 3) 반전 적용 (방향 반대)
        if (mapping.invert)
        {
            mappedValue = -mappedValue;
        }
        
        return mappedValue;
    }
    
    /// <summary>
    /// Unity MuJoCo → ROS2 값 변환 적용 메서드 (역변환)
    /// 
    /// 관절 값을 Unity MuJoCo 형식에서 ROS2 형식으로 역변환합니다.
    /// 변환 순서: 반전 해제 → 오프셋 제거 → 스케일 해제 (정방향의 역순)
    /// 
    /// 예시:
    /// - 반전 해제: -value → value
    /// - 오프셋 제거: value - offset
    /// - 스케일 해제: value / scaleFactor
    /// </summary>
    /// <param name="inputValue">입력 값 (Unity MuJoCo 형식)</param>
    /// <param name="mapping">매핑 정보 (변환 파라미터)</param>
    /// <returns>역변환된 값 (ROS2 형식)</returns>
    float ApplyInverseMapping(float inputValue, JointMapping mapping)
    {
        float mappedValue = inputValue;
        
        // 1) 반전 해제
        if (mapping.invert)
        {
            mappedValue = -mappedValue;
        }
        
        // 2) 오프셋 제거
        mappedValue -= mapping.offset;
        
        // 3) 스케일 해제 (0으로 나누기 방지)
        if (mapping.scaleFactor != 0f)
        {
            mappedValue /= mapping.scaleFactor;
        }
        
        return mappedValue;
    }
    
    /// <summary>
    /// ROS2 관절 이름을 인덱스로 변환하는 헬퍼 메서드
    /// 
    /// ROS2 표준 관절 순서에 따라 관절 이름을 배열 인덱스로 변환합니다.
    /// wx250s957 로봇팔의 표준 관절 순서를 사용합니다.
    /// </summary>
    /// <param name="jointName">변환할 관절 이름</param>
    /// <returns>ROS2 관절 배열의 인덱스 (0~6, -1: 찾을 수 없음)</returns>
    int GetROS2JointIndex(string jointName)
    {
        // ROS2 표준 관절 순서 (wx250s957 로봇팔 기준)
        string[] ros2JointNames = {
            "waist",        // 인덱스 0: 허리 회전
            "shoulder",     // 인덱스 1: 어깨 관절
            "elbow",        // 인덱스 2: 팔꿈치 관절
            "forearm_roll", // 인덱스 3: 팔뚝 회전
            "wrist_angle",  // 인덱스 4: 손목 각도
            "wrist_rotate", // 인덱스 5: 손목 회전
            "gripper"       // 인덱스 6: 그리퍼
        };
        
        for (int i = 0; i < ros2JointNames.Length; i++)
        {
            if (ros2JointNames[i] == jointName)
            {
                return i;
            }
        }
        
        return -1; // 찾을 수 없음
    }
    
    // === 그리퍼 관련 유틸리티 메서드들 ===
    // 그리퍼 관절의 특별한 처리를 위한 헬퍼 메서드들
    
    /// <summary>
    /// 그리퍼 손가락 동기화 설정 메서드
    /// 
    /// left_finger와 right_finger의 동기화를 활성화/비활성화합니다.
    /// 동기화가 활성화되면 두 손가락이 항상 동일한 값을 가집니다.
    /// </summary>
    /// <param name="enable">동기화 활성화 여부 (true: 동기화, false: 독립)</param>
    public void SetGripperSync(bool enable)
    {
        syncGripperFingers = enable;
    }
    
    /// <summary>
    /// 지정된 관절이 그리퍼 관절인지 확인하는 메서드
    /// 
    /// ROS2 관절 이름을 입력받아 해당 관절이 그리퍼 관절인지 확인합니다.
    /// 그리퍼 관절은 ROS2에서 1개이지만 Unity에서는 2개로 분할됩니다.
    /// </summary>
    /// <param name="jointName">확인할 관절 이름</param>
    /// <returns>그리퍼 관절 여부 (true: 그리퍼, false: 일반 관절)</returns>
    public bool IsGripperJoint(string jointName)
    {
        return mappingDict.ContainsKey(jointName) && mappingDict[jointName].isGripperJoint;
    }
    
    /// <summary>
    /// 그리퍼 관절 이름 목록을 반환하는 메서드
    /// 
    /// Unity MuJoCo에서 사용하는 그리퍼 관절 이름들을 리스트로 반환합니다.
    /// 일반적으로 "left_finger"와 "right_finger"를 포함합니다.
    /// </summary>
    /// <returns>그리퍼 관절 이름 리스트 (예: ["left_finger", "right_finger"])</returns>
    public List<string> GetGripperJointNames()
    {
        List<string> gripperJoints = new List<string>();
        foreach (var mapping in jointMappings)
        {
            if (mapping.isGripperJoint && !gripperJoints.Contains(mapping.unityJointName))
            {
                gripperJoints.Add(mapping.unityJointName);
            }
        }
        return gripperJoints;
    }
    
    /// <summary>
    /// Unity GUI를 통한 매핑 정보 표시 및 제어 메서드
    /// 
    /// 런타임에서 매핑 상태를 확인하고 그리퍼 동기화를 토글할 수 있습니다.
    /// 화면 우측 상단에 매핑 정보를 표시하며, 디버깅과 설정 변경에 유용합니다.
    /// 
    /// 표시 정보:
    /// - 기본 관절 매핑 (ROS2 ↔ Unity MuJoCo)
    /// - 그리퍼 매핑 (ROS2 1개 → Unity 2개)
    /// - 현재 설정 상태 (그리퍼 매핑, 동기화)
    /// - 그리퍼 동기화 토글 버튼
    /// </summary>
    void OnGUI()
    {
        if (!showMappingInfo) return;
        
        // GUI 영역 설정 (화면 우측 상단)
        GUILayout.BeginArea(new Rect(370, 10, 350, 450));
        GUILayout.Label("Unity Joint Mapping Info", GUI.skin.box);
        
        // === 기본 관절 매핑 정보 표시 ===
        GUILayout.Label("기본 관절 매핑:");
        foreach (var mapping in jointMappings)
        {
            if (!mapping.isGripperJoint)
            {
                GUILayout.Label($"  {mapping.ros2JointName} → {mapping.unityJointName}");
            }
        }
        
        GUILayout.Space(10);
        
        // === 그리퍼 매핑 정보 표시 ===
        GUILayout.Label("그리퍼 매핑:");
        foreach (var mapping in jointMappings)
        {
            if (mapping.isGripperJoint)
            {
                GUILayout.Label($"  {mapping.ros2JointName} → {mapping.unityJointName}");
            }
        }
        
        GUILayout.Space(10);
        
        // === 현재 설정 상태 표시 ===
        GUILayout.Label($"그리퍼 매핑: {(enableGripperMapping ? "ON" : "OFF")}");
        GUILayout.Label($"그리퍼 동기화: {(syncGripperFingers ? "ON" : "OFF")}");
        
        GUILayout.Space(10);
        
        // === 그리퍼 동기화 토글 버튼 ===
        if (GUILayout.Button("그리퍼 동기화 토글"))
        {
            SetGripperSync(!syncGripperFingers);
        }
        
        GUILayout.EndArea();
    }
}