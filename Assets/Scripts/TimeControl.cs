using UnityEngine;
using UnityEngine.UI;

public class TimeControl : MonoBehaviour
{
    public Button startButton;
    public Button stopButton;

    void Start()
    {
        // スタートボタンのクリックイベントを設定
        startButton.onClick.AddListener(StartTime);

        // ストップボタンのクリックイベントを設定
        stopButton.onClick.AddListener(StopTime);
    }

    // 内部時間を進める
    void StartTime()
    {
        Time.timeScale = 1f;  // 通常の速度で時間を進める
    }

    // 内部時間を止める
    void StopTime()
    {
        Time.timeScale = 0f;  // 時間を停止
    }
}