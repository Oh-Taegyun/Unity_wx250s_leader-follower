// Assets/RobotIntrospect/MjRobotIntrospector.cs
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using UnityEngine;

// MuJoCo for Unity 일반적인 네임스페이스/타입
// (필요 시 프로젝트에 맞게 네임스페이스 또는 클래스명을 조정하세요)
using Mujoco;  // <- 프로젝트에 따라 맞춰주세요

[Serializable]
public class JointInfo
{
    public string name;
    public string type;
    public string parentBody;
    public string childBody;
    public bool limited;
    public Vector2 range;      // (min, max), hinge/slide에 해당
    public Vector3 axis;       // hinge/slide 축
    public List<string> actuators = new();
}

[Serializable]
public class BodyInfo
{
    public string name;
    public string path;
    public string parent;
}

[Serializable]
public class RobotIntrospectionResult
{
    public string robotRootName;
    public List<BodyInfo> bodies = new();
    public List<JointInfo> joints = new();
}

public class MjRobotIntrospector : MonoBehaviour
{
    [Tooltip("로봇 루트 Transform (trossen_wx250s의 최상위 오브젝트)")]
    public Transform robotRoot;

    [Tooltip("결과를 JSON으로 저장할지 여부")]
    public bool writeJson = true;

    [Tooltip("Assets 하위 저장 경로")]
    public string saveDirInAssets = "Assets/RobotIntrospect";

    private void Start()
    {
        if (robotRoot == null)
        {
            Debug.LogError("[MjRobotIntrospector] robotRoot가 지정되지 않았습니다.");
            return;
        }

        var result = new RobotIntrospectionResult
        {
            robotRootName = robotRoot.name
        };

        // 1) 모든 링크(Body) 수집
        var bodies = robotRoot.GetComponentsInChildren<MjBody>(includeInactive: true);
        foreach (var b in bodies)
        {
            var bodyInfo = new BodyInfo
            {
                name = b.name,
                path = GetFullPath(b.transform),
                parent = b.transform.parent ? b.transform.parent.name : "(none)"
            };
            result.bodies.Add(bodyInfo);
        }

        // 2) 모든 관절 수집 (hinge / slide / ball / free)
        //    필요 시 프로젝트 타입명에 맞게 수정
        //    Unity용 MuJoCo 플러그인의 C# 네임스페이스/컴포넌트는 보통 Mujoco (예: MjBody, MjHingeJoint, MjSlideJoint, MjBallJoint, MjFreeJoint, MjActuator)를 사용한단느 가정
        var hingeJoints = robotRoot.GetComponentsInChildren<MjHingeJoint>(true);
        var slideJoints = robotRoot.GetComponentsInChildren<MjSlideJoint>(true);
        var ballJoints  = robotRoot.GetComponentsInChildren<MjBallJoint>(true);
        var freeJoints  = robotRoot.GetComponentsInChildren<MjFreeJoint>(true);

        // 헬퍼: 액추에이터 맵 (joint -> actuator 리스트)
        // 일반적으로 MjActuator가 특정 조인트를 타겟으로 가리킵니다.
        var actuatorMap = BuildActuatorMap(robotRoot);

        // Hinge
        foreach (var j in hingeJoints)
            result.joints.Add(ToJointInfo_HingeOrSlide(j, "hinge", actuatorMap));

        // Slide
        foreach (var j in slideJoints)
            result.joints.Add(ToJointInfo_HingeOrSlide(j, "slide", actuatorMap));

        // Ball
        foreach (var j in ballJoints)
            result.joints.Add(ToJointInfo_BallOrFree(j, "ball", actuatorMap));

        // Free
        foreach (var j in freeJoints)
            result.joints.Add(ToJointInfo_BallOrFree(j, "free", actuatorMap));

        // 3) 콘솔 출력(요약)
        Debug.Log($"[MjRobotIntrospector] Bodies: {result.bodies.Count}, Joints: {result.joints.Count}");
        foreach (var ji in result.joints)
        {
            var rangeStr = ji.type is "hinge" or "slide" ? $" range=({ji.range.x:F3},{ji.range.y:F3}) limited={ji.limited}" : "";
            var axisStr  = ji.type is "hinge" or "slide" ? $" axis={ji.axis}" : "";
            var actStr   = ji.actuators.Count > 0 ? $" actuators=[{string.Join(", ", ji.actuators)}]" : "";
            Debug.Log($"  - {ji.name} [{ji.type}] parent={ji.parentBody} child={ji.childBody}{axisStr}{rangeStr}{actStr}");
        }

        // 4) JSON 저장
        if (writeJson)
        {
            Directory.CreateDirectory(saveDirInAssets);
            var savePath = Path.Combine(saveDirInAssets, $"{robotRoot.name}_introspection.json");
            var json = JsonUtility.ToJson(result, prettyPrint: true);
            File.WriteAllText(savePath, json);
            Debug.Log($"[MjRobotIntrospector] JSON 저장: {savePath}");
            #if UNITY_EDITOR
            UnityEditor.AssetDatabase.Refresh();
            #endif
        }
    }

    private static string GetFullPath(Transform t)
    {
        var stack = new Stack<string>();
        while (t != null)
        {
            stack.Push(t.name);
            t = t.parent;
        }
        return string.Join("/", stack);
    }

    private static Dictionary<Component, List<string>> BuildActuatorMap(Transform root)
    {
        var dict = new Dictionary<Component, List<string>>();
        var actuators = root.GetComponentsInChildren<MjActuator>(true);
        foreach (var a in actuators)
        {
            // 일반적으로 MjActuator에는 target joint에 대한 참조가 있습니다.
            // 필드/프로퍼티 이름이 패키지 버전에 따라 다를 수 있어
            // 가장 흔한 'Joint' / 'joint'를 우선 시도하고, 리플렉션 fallback을 둡니다.
            Component targetJoint = null;

            // 1) 직관적 프로퍼티 시도
            var jointProp = a.GetType().GetProperty("Joint");
            if (jointProp != null) targetJoint = jointProp.GetValue(a) as Component;

            // 2) 필드 시도
            if (targetJoint == null)
            {
                var jointField = a.GetType().GetField("joint");
                if (jointField != null) targetJoint = jointField.GetValue(a) as Component;
            }

            // 3) 리플렉션 백업: 첫 번째로 발견되는 Mj*Joint 타입 참조
            if (targetJoint == null)
            {
                var fields = a.GetType().GetFields();
                foreach (var f in fields)
                {
                    if (f.FieldType.Name.Contains("Joint") && f.GetValue(a) is Component c)
                    {
                        targetJoint = c;
                        break;
                    }
                }
            }

            if (targetJoint != null)
            {
                if (!dict.ContainsKey(targetJoint))
                    dict[targetJoint] = new List<string>();
                dict[targetJoint].Add(a.name);
            }
        }
        return dict;
    }

    private static JointInfo ToJointInfo_HingeOrSlide(Component j, string type, Dictionary<Component, List<string>> actuatorMap)
    {
        // j는 MjHingeJoint 또는 MjSlideJoint
        var t = j.transform;
        var parentBody = t.parent ? t.parent.GetComponent<MjBody>() : null;
        var childBody  = t.GetComponentInParent<MjBody>();

        bool limited = false;
        Vector2 range = default;
        Vector3 axis = Vector3.zero;

        // 일반적으로 Hinge/Slide에는 limited(bool), range(Vector2), axis(Vector3) 필드가 존재
        var jType = j.GetType();

        var limitedField = jType.GetField("limited");
        if (limitedField != null) limited = (bool)limitedField.GetValue(j);

        var rangeField = jType.GetField("range");
        if (rangeField != null) range = (Vector2)rangeField.GetValue(j);

        var axisField = jType.GetField("axis");
        if (axisField != null) axis = (Vector3)axisField.GetValue(j);

        var info = new JointInfo
        {
            name = j.name,
            type = type,
            parentBody = parentBody ? parentBody.name : "(none)",
            childBody  = childBody  ? childBody.name  : j.name,
            limited = limited,
            range = range,
            axis = axis,
            actuators = actuatorMap.TryGetValue(j, out var list) ? new List<string>(list) : new List<string>()
        };
        return info;
    }

    private static JointInfo ToJointInfo_BallOrFree(Component j, string type, Dictionary<Component, List<string>> actuatorMap)
    {
        var t = j.transform;
        var parentBody = t.parent ? t.parent.GetComponent<MjBody>() : null;
        var childBody  = t.GetComponentInParent<MjBody>();

        var info = new JointInfo
        {
            name = j.name,
            type = type,
            parentBody = parentBody ? parentBody.name : "(none)",
            childBody  = childBody  ? childBody.name  : j.name,
            limited = false,
            range = Vector2.zero,
            axis = Vector3.zero,
            actuators = actuatorMap.TryGetValue(j, out var list) ? new List<string>(list) : new List<string>()
        };
        return info;
    }
}
