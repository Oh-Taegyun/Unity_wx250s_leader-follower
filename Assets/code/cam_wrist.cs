using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;
using System;

[RequireComponent(typeof(Camera))]
public class cam_wrist : MonoBehaviour
{
    [Header("ROS")]
    public string rawTopicName = "/camera/image_raw";
    public string compressedTopicName = "/camera/image_raw/compressed";
    public string encoding = "rgb8"; // "mono8", "bgr8" 등도 가능

    [Header("Publish Rate (Hz)")]
    public float publishHz = 15f;

    [Header("Capture")]
    public int width = 640;
    public int height = 480;

    Camera _cam;
    ROSConnection _ros;
    RenderTexture _rt;
    Texture2D _tex2D;
    float _nextPublishTime;

    void Start()
    {
        _cam = GetComponent<Camera>();
        _ros = ROSConnection.GetOrCreateInstance();
        _ros.RegisterPublisher<ImageMsg>(rawTopicName);
        _ros.RegisterPublisher<CompressedImageMsg>(compressedTopicName);

        _rt = new RenderTexture(width, height, 0, RenderTextureFormat.ARGB32);
        _rt.Create();

        // RGB24로 만들어두면 step 계산이 쉬움(3바이트/픽셀)
        _tex2D = new Texture2D(width, height, TextureFormat.RGB24, false);
    }

    void OnDestroy()
    {
        if (_rt != null) _rt.Release();
        Destroy(_rt);
        Destroy(_tex2D);
    }

    void LateUpdate()
    {
        if (Time.time < _nextPublishTime) return;
        _nextPublishTime = Time.time + (1f / Mathf.Max(1f, publishHz));

        // 카메라를 RenderTexture로 렌더
        var prevRT = RenderTexture.active;
        _cam.targetTexture = _rt;
        _cam.Render();
        RenderTexture.active = _rt;

        // RT -> Texture2D 복사 (RGB24)
        _tex2D.ReadPixels(new Rect(0, 0, width, height), 0, 0, false);
        _tex2D.Apply(false, false);

        // 정리
        _cam.targetTexture = null;
        RenderTexture.active = prevRT;

        // 픽셀 데이터 추출 (RGB24) - 뒤집기 적용
        byte[] rgbBytes = _tex2D.GetRawTextureData(); // size = width*height*3

        // 이미지를 수직으로 뒤집기
        byte[] flippedBytes = new byte[rgbBytes.Length];
        int bytesPerRow = width * 3; // RGB24이므로 3바이트/픽셀

        for (int y = 0; y < height; y++)
        {
            int sourceRow = y;
            int destRow = height - 1 - y; // 뒤집기
            
            for (int x = 0; x < width; x++)
            {
                int sourceIndex = (sourceRow * width + x) * 3;
                int destIndex = (destRow * width + x) * 3;
                
                // RGB 3바이트 복사
                flippedBytes[destIndex] = rgbBytes[sourceIndex];     // R
                flippedBytes[destIndex + 1] = rgbBytes[sourceIndex + 1]; // G
                flippedBytes[destIndex + 2] = rgbBytes[sourceIndex + 2]; // B
            }
        }

        // Raw 이미지 메시지 구성
        var header = new HeaderMsg();
        // stamp는 Endpoint가 덮어쓰는 경우가 많지만, 명시하고 싶으면 Unity 시간으로 채워도 됨.
        // header.stamp = new RosMessageTypes.BuiltinInterfaces.TimeMsg(sec, nanosec);
        header.frame_id = "camera_color_optical_frame";

        var rawMsg = new ImageMsg
        {
            header = header,
            height = (uint)height,
            width = (uint)width,
            encoding = encoding,
            is_bigendian = 0,
            step = (uint)(width * 3),
            data = flippedBytes
        };

        // JPEG로 압축
        byte[] jpegBytes = _tex2D.EncodeToJPG(80); // 품질 80%

        var compressedMsg = new CompressedImageMsg
        {
            header = header,
            format = "jpeg",
            data = jpegBytes
        };

        // 두 토픽으로 동시 발행
        _ros.Publish(rawTopicName, rawMsg);
        _ros.Publish(compressedTopicName, compressedMsg);
    }
}
