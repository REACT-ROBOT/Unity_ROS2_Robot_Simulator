using UnityEngine;

public class CameraController : MonoBehaviour
{
    public float moveSpeed = 0.1f;  // 移動速度
    public float rotationSpeed = 0.1f;  // 回転速度
    public float zoomSpeed = 10f;  // ズーム速度

    private Camera cam;
    private Vector3 lastMousePosition;
    private Vector3 lastMousePositionForRotation;

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
        if (Input.GetMouseButtonDown(1))  // 右クリックを押したとき
        {
            lastMousePositionForRotation = Input.mousePosition;
        }
        if (Input.GetMouseButton(1))  // 右クリック
        {
            // マウスの移動に基づいてカメラを回転
            Vector3 mouseDelta = Input.mousePosition - lastMousePositionForRotation;
            lastMousePositionForRotation = Input.mousePosition;

            float horizontalRotation = mouseDelta.x * rotationSpeed;
            float verticalRotation = -mouseDelta.y * rotationSpeed;

            transform.Rotate(Vector3.up, horizontalRotation, Space.World);
            transform.Rotate(Vector3.right, verticalRotation, Space.Self);
        }

        // マウスのホイールでズームイン・ズームアウト
        float scroll = Input.GetAxis("Mouse ScrollWheel");
        if (Mathf.Abs(scroll) > 0.0001f)
        {
            Vector3 forward = cam.transform.forward;
            cam.transform.position += forward * scroll * zoomSpeed;
        }

        // 中ボタンまたは右クリックをドラッグして視点移動
        if (Input.GetMouseButtonDown(2))  // 中ボタン(マウスホイール)を押したとき
        {
            lastMousePosition = Input.mousePosition;
        }
        if (Input.GetMouseButton(2))  // 中ボタン(マウスホイール)を押しながら移動
        {
            Vector3 mouseDelta = Input.mousePosition - lastMousePosition;
            lastMousePosition = Input.mousePosition;

            // 移動をカメラの空間で適用
            Vector3 move = new Vector3(-mouseDelta.x, -mouseDelta.y, 0) * moveSpeed * Time.unscaledDeltaTime;
            transform.Translate(move, Space.Self);
        }
    }
}
