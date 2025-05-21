// FrameRateController.cs
using UnityEngine;
using TMPro;  // ← 必ず追加！

public class FrameRateController : MonoBehaviour
{
    [Header("TMP の Input Field をアサイン")]
    public TMP_InputField frameRateInput;

    void Start()
    {
        // 起動時にデフォルトの targetFrameRate をセット
        // ここでは 10 FPS に設定。-1 は無制限。
        Application.targetFrameRate = 10;
        // 起動時に現在の targetFrameRate を表示（-1 はプラットフォームのデフォルト）
        frameRateInput.text = Application.targetFrameRate.ToString();
        // フォーカス外し or Enter 押下で呼び出し
        frameRateInput.onEndEdit.AddListener(OnFrameRateInputEnd);
    }

    private void OnFrameRateInputEnd(string value)
    {
        if (int.TryParse(value, out int fps))
        {
            // 1 以上の値のみ受け付け。それ以外はデフォルト（-1＝無制限）に戻すなら fps = -1;
            fps = Mathf.Max(1, fps);
            Application.targetFrameRate = fps;
            Debug.Log($"[FrameRateController] Application.targetFrameRate = {fps}");
        }
        else
        {
            // 不正入力時は元の値に戻す
            frameRateInput.text = Application.targetFrameRate.ToString();
        }
    }
}
