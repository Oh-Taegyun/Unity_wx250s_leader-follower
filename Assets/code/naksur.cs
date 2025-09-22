using UnityEngine;

public class naksur : MonoBehaviour
{
    private void OnTriggerEnter(Collider other)
    {
        // "erase"라는 이름의 오브젝트와 부딪히면 제거
        if (other.gameObject.name == "erase")
        {
            Destroy(gameObject); // 자기 자신(quad)을 없앰
        }
    }
}
