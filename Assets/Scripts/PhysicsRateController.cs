using UnityEngine;
using TMPro;

/// <summary>
/// 物理演算レート (Time.fixedDeltaTime) をサイドバーから変えられるようにする
/// 入力欄。FrameRateController の行 (SetTimeFreq) を起動時に複製して、同じ
/// 見た目の "Physics Rate[Hz]" 行をすぐ下に作る。
/// </summary>
/// <remarks>
/// 物理レートを上げると高速走行時のホイールスリップが大きく改善する
/// (docs/URDF-Collision-Material.md: 1.5 m/s で 50 Hz → 74% スリップ、
/// 200 Hz → 7%)。一方で実行中の変更は決定性を壊し、サーボモデルの
/// 伝達剛性の安定上限も物理ステップに紐づいている
/// (docs/Known-Limitations.md) ので、値は 10..1000 Hz にクランプし、
/// 変更時にはログで注意を出す。
/// </remarks>
public class PhysicsRateController : MonoBehaviour
{
    public const int MinPhysicsHz = 10;
    public const int MaxPhysicsHz = 1000;

    [Header("TMP の Integer Input Field をアサイン")]
    public TMP_InputField physicsRateInput;

    /// <summary>
    /// FrameRateController の行を複製して UI を組み立てる。シーン YAML の
    /// 手編集は事故りやすいので、起動時にコードから作る (ClockPub と同じ方針)。
    /// </summary>
    [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.AfterSceneLoad)]
    private static void Bootstrap()
    {
        if (Object.FindFirstObjectByType<PhysicsRateController>(FindObjectsInactive.Include) != null)
        {
            return;  // シーンに手で置かれていたら何もしない
        }

        FrameRateController frameCtl =
            Object.FindFirstObjectByType<FrameRateController>(FindObjectsInactive.Include);
        if (frameCtl == null)
        {
            Debug.LogWarning("[PhysicsRateController] FrameRateController が見つからないので物理レート入力欄を作れない");
            return;
        }

        // ラベル + 入力欄の行ごと複製し、フレームレート行の直下に並べる。
        // 親 (Content) は VerticalLayoutGroup なので位置は自動で決まる。
        GameObject row = Object.Instantiate(frameCtl.gameObject, frameCtl.transform.parent);
        row.name = "SetPhysicsFreq";
        row.transform.SetSiblingIndex(frameCtl.transform.GetSiblingIndex() + 1);

        // 複製された FrameRateController は残すと Start で targetFrameRate を
        // 二重に触るので外す。入力欄への参照 (複製内へ張り替え済み) だけ回収する。
        FrameRateController clonedCtl = row.GetComponent<FrameRateController>();
        TMP_InputField input = clonedCtl != null ? clonedCtl.frameRateInput : null;
        if (input == null)
        {
            input = row.GetComponentInChildren<TMP_InputField>(true);
        }
        if (clonedCtl != null)
        {
            Object.DestroyImmediate(clonedCtl);
        }
        if (input == null)
        {
            Debug.LogWarning("[PhysicsRateController] 複製した行に TMP_InputField が無い");
            Object.Destroy(row);
            return;
        }

        // 見出しを差し替える。複製なので子の名前は元の行 (TimeFreqTitle) と同じ。
        Transform title = row.transform.Find("TimeFreqTitle");
        TMP_Text titleText = title != null ? title.GetComponent<TMP_Text>() : null;
        if (titleText == null)
        {
            // 名前が変わっていたら「入力欄の外にある最初のラベル」を探す。
            foreach (TMP_Text label in row.GetComponentsInChildren<TMP_Text>(true))
            {
                if (!label.transform.IsChildOf(input.transform))
                {
                    titleText = label;
                    break;
                }
            }
        }
        if (titleText != null)
        {
            titleText.text = "Physics Rate[Hz]";

            // "Physics Rate[Hz]" はフォントサイズ 20 で約 155 px あり、元の行の
            // ラベル幅 150 px では 2 行に折り返す。ラベル幅はパネル幅から決まらず
            // (Content の VerticalLayoutGroup も行の HorizontalLayoutGroup も
            // ChildControlWidth が無効で、150 px 固定)、サイドバーを広げても
            // 直らないので、この複製行の中だけで幅を配り直す:
            // ラベル 150→180 / 入力欄 100→70。合計 250 は行幅と同じままなので
            // 見た目の並びは元の行と揃う。入力欄は最大 4 桁 (1000) をポイント 14
            // で表示するだけなので 70 px で足りる。元の Frame Rate 行は触らない。
            RectTransform titleRect = titleText.GetComponent<RectTransform>();
            RectTransform inputRect = input.GetComponent<RectTransform>();
            if (titleRect != null && inputRect != null)
            {
                titleRect.sizeDelta = new Vector2(180f, titleRect.sizeDelta.y);
                inputRect.sizeDelta = new Vector2(70f, inputRect.sizeDelta.y);
            }
        }

        input.onEndEdit.RemoveAllListeners();  // 念のため。ランタイム登録分は複製されない

        PhysicsRateController controller = row.AddComponent<PhysicsRateController>();
        controller.physicsRateInput = input;
    }

    void Start()
    {
        if (physicsRateInput == null)
        {
            Debug.LogError("PhysicsRateController: physicsRateInput がアサインされていません。");
            enabled = false;
            return;
        }
        // 起動時に現在の物理レートを表示 (既定 50 Hz、settings があればその値)
        RefreshDisplayedValue();
        // フォーカス外し or Enter 押下で呼び出し
        physicsRateInput.onEndEdit.AddListener(OnPhysicsRateInputEnd);
    }

    /// <summary>入力欄の表示を現在の Time.fixedDeltaTime に合わせ直す。</summary>
    public void RefreshDisplayedValue()
    {
        if (physicsRateInput != null)
        {
            physicsRateInput.text = Mathf.RoundToInt(1f / Time.fixedDeltaTime).ToString();
        }
    }

    private void OnPhysicsRateInputEnd(string value)
    {
        if (int.TryParse(value, out int hz))
        {
            hz = Mathf.Clamp(hz, MinPhysicsHz, MaxPhysicsHz);
            Time.fixedDeltaTime = 1f / hz;
            physicsRateInput.text = hz.ToString();  // クランプ結果を見せる
            Debug.Log($"[PhysicsRateController] physics rate = {hz} Hz (fixedDeltaTime = {Time.fixedDeltaTime:F6} s)。" +
                      "実行中の変更は決定性を壊し、サーボモデルの安定余裕 (伝達剛性の上限) も変わる点に注意");
        }
        else
        {
            // 不正入力時は元の値に戻す
            RefreshDisplayedValue();
        }
    }
}
