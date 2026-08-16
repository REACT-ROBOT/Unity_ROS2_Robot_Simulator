using UnityEngine;
using UnityEngine.UI;
using TMPro;

/// <summary>
/// 3D 画面の上に重ねる TMP ラベルを、背景がどんな色でも読めるようにする。
/// 文字に縁取りを入れ、背後に半透明の暗い板を敷く。
/// </summary>
/// <remarks>
/// FPS と RTF の表示は白文字をシーンへ直接重ねているだけなので、明るい床や
/// 白いロボットの上に来ると輪郭が溶けて読めない。縁取りだけでも輪郭は出るが、
/// 細い書体では白地の上でまだ弱いので、板も併せて敷く。
/// </remarks>
public static class OverlayLabelStyle
{
    /// <summary>縁取りの色。</summary>
    public static readonly Color OutlineColor = new Color(0f, 0f, 0f, 1f);

    /// <summary>
    /// 縁取りの太さ (0..1)。TMP は文字の太さに対する比で持つので、
    /// フォントサイズが変わっても見た目の比率は保たれる。
    /// </summary>
    public const float OutlineWidth = 0.2f;

    /// <summary>板の色。黒の 55% で、白文字とも暗い背景とも喧嘩しない。</summary>
    public static readonly Color BackdropColor = new Color(0f, 0f, 0f, 0.55f);

    /// <summary>縁取りと背景板の両方を付ける。</summary>
    public static void Apply(TMP_Text label)
    {
        if (label == null)
        {
            return;
        }
        ApplyOutline(label);
        LabelBackdrop.AttachTo(label);
    }

    /// <summary>文字に縁取りを入れる。</summary>
    /// <remarks>
    /// fontSharedMaterial ではなく fontMaterial を触ること。前者はフォント
    /// アセットの共有マテリアルで、書き換えると同じフォントを使う他の
    /// ラベル (サイドバーの入力欄など) まで縁取りされる。fontMaterial は
    /// 参照した時点でインスタンスが作られるので、このラベルだけに効く。
    /// </remarks>
    private static void ApplyOutline(TMP_Text label)
    {
        Material material = label.fontMaterial;
        if (material == null)
        {
            return;
        }
        material.EnableKeyword(ShaderUtilities.Keyword_Outline);
        material.SetColor(ShaderUtilities.ID_OutlineColor, OutlineColor);
        material.SetFloat(ShaderUtilities.ID_OutlineWidth, OutlineWidth);
    }
}

/// <summary>
/// TMP ラベルの背後に敷く半透明の板。文字の実寸に合わせて追従する。
/// </summary>
/// <remarks>
/// 板はラベルと同じ親の下に、ラベルより手前の兄弟位置へ入れる。uGUI は
/// ヒエラルキーの順に描くので、これで必ずラベルの背後になる。ラベルの子に
/// すると手前に来てしまい、文字が隠れる。
///
/// 大きさをラベルの RectTransform に合わせてはいけない。FPS ラベルは幅 200 px
/// 固定で確保されているのに文字は "FPS: 60.0" ぶんしか無く、板だけが間延びする。
/// さらに RTF ラベルは FPS ラベルの右端 +10 px に置かれる (RtfDisplay.Bootstrap)
/// ので、左右にはみ出す量が 5 px を超えると隣の板と端が重なる。
/// そこで TMP の textBounds (実際に描かれた文字の範囲) に合わせる。
/// </remarks>
[RequireComponent(typeof(RectTransform))]
public class LabelBackdrop : MonoBehaviour
{
    /// <summary>板が文字の外へはみ出す量 [px]。</summary>
    public static readonly Vector2 Padding = new Vector2(6f, 2f);

    public TMP_Text label;

    private RectTransform m_Rect;
    private string m_LastText;
    private Vector2 m_LastSize;

    /// <summary>ラベルの背後に板を作って付ける。すでにあれば作らない。</summary>
    public static LabelBackdrop AttachTo(TMP_Text label)
    {
        RectTransform labelRect = label != null ? label.rectTransform : null;
        if (labelRect == null || labelRect.parent == null)
        {
            return null;
        }

        string backdropName = label.gameObject.name + "_Backdrop";
        Transform existing = labelRect.parent.Find(backdropName);
        if (existing != null)
        {
            return existing.GetComponent<LabelBackdrop>();
        }

        var go = new GameObject(backdropName, typeof(RectTransform), typeof(Image));
        var rect = go.GetComponent<RectTransform>();
        rect.SetParent(labelRect.parent, false);
        // 中心を合わせて置きたいので、アンカーも軸も中央に固定する。
        // (ラベル側の設定は引き継がない。位置はワールド座標で毎回合わせ直す)
        rect.anchorMin = new Vector2(0.5f, 0.5f);
        rect.anchorMax = new Vector2(0.5f, 0.5f);
        rect.pivot = new Vector2(0.5f, 0.5f);

        var image = go.GetComponent<Image>();
        image.color = OverlayLabelStyle.BackdropColor;
        image.raycastTarget = false;  // 下のボタンのクリックを奪わない

        // ラベルの手前の兄弟位置へ入れる = 描画はラベルより先 = 背後に来る。
        rect.SetSiblingIndex(labelRect.GetSiblingIndex());

        var backdrop = go.AddComponent<LabelBackdrop>();
        backdrop.label = label;
        backdrop.m_Rect = rect;
        backdrop.Fit();
        return backdrop;
    }

    void Awake()
    {
        if (m_Rect == null)
        {
            m_Rect = GetComponent<RectTransform>();
        }
    }

    void LateUpdate()
    {
        if (label == null)
        {
            return;
        }
        // 文字が変わったときだけ測り直す。FPS/RTF は 0.5〜1 秒に 1 回しか
        // 変わらないので、毎フレーム測る必要はない。
        if (label.text == m_LastText)
        {
            return;
        }
        m_LastText = label.text;
        Fit();
    }

    /// <summary>板を文字の実寸へ合わせる。</summary>
    private void Fit()
    {
        if (label == null || m_Rect == null)
        {
            return;
        }

        // 先にメッシュを作り直させてから測る。TMP がメッシュを再生成するのは
        // 自分の更新タイミングで、この LateUpdate より後になり得る。順序に任せると
        // 「1 つ前のテキストの寸法」で板を作ってしまい、桁が増えたとき
        // (9.9 -> 60.0) に文字が板からはみ出す。
        label.ForceMeshUpdate();

        Bounds bounds = label.textBounds;
        Vector2 textSize = new Vector2(bounds.size.x, bounds.size.y);

        // まだメッシュが作られていないと 0 になる。その場合は前回の大きさを保つ
        // (初回は板を出さない)。次にテキストが変わったときに測り直される。
        if (textSize.x <= 0f || textSize.y <= 0f)
        {
            m_LastText = null;  // 次のフレームでもう一度試す
            m_Rect.sizeDelta = m_LastSize;
            return;
        }

        m_LastSize = textSize + Padding * 2f;
        m_Rect.sizeDelta = m_LastSize;
        // 文字の中心 (ラベルのローカル座標) をワールドへ移してから合わせる。
        // ラベル側の揃え方 (左寄せ・中央寄せ) に関わらず文字の上に乗る。
        m_Rect.position = label.rectTransform.TransformPoint(bounds.center);
    }
}
