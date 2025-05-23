using UnityEngine;
using TMPro;  // ここを追加

public class FpsDisplayTMP : MonoBehaviour
{
    [Tooltip("FPS を表示する TextMeshProUGUI コンポーネント")]
    [SerializeField] private TMP_Text fpsText;
    // または
    // [SerializeField] private TextMeshProUGUI fpsText;

    [Tooltip("FPS 更新間隔（秒）")]
    [SerializeField] private float updateInterval = 0.5f;

    private float accum    = 0f;   // フレームレートの合計
    private int   frames   = 0;    // 計測フレーム数
    private float timeLeft;        // 次の更新までの残り時間

    void Start()
    {
        if (fpsText == null)
        {
            Debug.LogError("FpsDisplayTMP: fpsText がアサインされていません。");
            enabled = false;
            return;
        }
        timeLeft = updateInterval;
    }

    void Update()
    {
        // Time.timeScale に影響されないデルタタイムで計測
        timeLeft  -= Time.unscaledDeltaTime;
        accum     += 1f / Time.unscaledDeltaTime;
        frames++;

        if (timeLeft <= 0f)
        {
            float fps = accum / frames;
            // TMP_Text でも .text プロパティは同じ
            fpsText.text = $"FPS: {fps:F1}";

            // リセット
            timeLeft = updateInterval;
            accum    = 0f;
            frames   = 0;
        }
    }
}
