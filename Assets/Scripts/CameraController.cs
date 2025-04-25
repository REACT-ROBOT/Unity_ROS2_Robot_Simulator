using UnityEngine;

public class CameraController : MonoBehaviour
{
    public float moveSpeed = 0.1f;  // 移動速度
    public float rotationSpeed = 5f;  // 回転速度
    public float zoomSpeed = 10f;  // ズーム速度
    public float minZoom = 5f;  // ズームの最小値
    public float maxZoom = 50f;  // ズームの最大値

    private Camera cam;
    private Vector3 lastMousePosition;

    void Start()
    {
        cam = Camera.main;
    }

    void Update()
    {
        HandleMouseInput();
    }

    // マウス入力による視点移動、回転、ズームを処理
    private void HandleMouseInput()
    {
        // マウスの右ボタンを使って視点を回転
        if (Input.GetMouseButton(1))  // 右クリック
        {
            // マウスの移動に基づいてカメラを回転
            float horizontalRotation = Input.GetAxis("Mouse X") * rotationSpeed;
            float verticalRotation = -Input.GetAxis("Mouse Y") * rotationSpeed;

            transform.Rotate(Vector3.up, horizontalRotation, Space.World);
            transform.Rotate(Vector3.right, verticalRotation, Space.Self);
        }

        // マウスのホイールでズームイン・ズームアウト
        float scroll = Input.GetAxis("Mouse ScrollWheel");
        float newZoom = cam.fieldOfView - scroll * zoomSpeed;
        cam.fieldOfView = Mathf.Clamp(newZoom, minZoom, maxZoom);

        // 中ボタンまたは右クリックをドラッグして視点移動
        if (Input.GetMouseButton(2))  // 中ボタン(マウスホイール)を押しながら移動
        {
            Vector3 mouseDelta = Input.mousePosition - lastMousePosition;
            lastMousePosition = Input.mousePosition;

            // 移動をカメラの空間で適用
            Vector3 move = new Vector3(-mouseDelta.x, -mouseDelta.y, 0) * moveSpeed * Time.unscaledDeltaTime;
            transform.Translate(move, Space.Self);
        }
        else
        {
            lastMousePosition = Input.mousePosition;
        }
    }
}
