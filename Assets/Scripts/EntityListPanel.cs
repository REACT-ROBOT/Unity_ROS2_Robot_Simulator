using System.Collections;
using System.Collections.Generic;
using TMPro;
using UnityEngine;
using UnityEngine.EventSystems;
using UnityEngine.UI;

/// <summary>
/// スポーン済みエンティティの一覧と、選択したエンティティの関節スライダを出すパネル。
/// SimulationControl.Awake (SimulationEntityPanel.cs) からコードで取り付けられる。
/// </summary>
/// <remarks>
/// UI はすべてコードから組み立てる。シーンにはスライダが存在しないので、
/// Background / Fill / Handle を持つ標準構成の Slider を自前で作り、見た目は
/// 既存 UI と同じ部品 (Resources の ListItemPrehab の UISprite と TMP フォント、
/// 選択色のシアン) を使い回す。
///
/// 配置と見た目は既存の右側 UI (景観・設定サイドバー) に合わせる:
///   - 開閉ボタンは右下のボタン列 (Add -125 / Save -75 / Load -25、40x40・
///     間隔 50) の続きとして (-175, 25) に置く。アイコンは
///     Resources/EntityButton.png (ロボットの絵)。読めなければ文字 "Entities" の
///     ボタンに落とす。
///   - パネルは SideBarPanel (設定サイドバー) と同じ右端スライド。幅・背景
///     (組み込み Background スプライト、白 α0.392)・スライド時間・表示/非表示の
///     anchoredPosition 規約 (表示 -幅/2、非表示 +幅) は実行時に
///     SidebarController からコピーする。既存の 2 枚 (設定 300 / 景観 350) と
///     同じく、パネル同士の重なりは描画順 (後に作ったものが手前) に任せる。
///   - 閉じるのは既存パネルと同じ作法: パネル右上 (-25,-25) の 40x40 の
///     CloseButton。アイコンは既存サイドバーの CloseButton からコピーする。
///     開くボタンは AddButton / HamburgerButton と同じくパネルより先に作る =
///     開いている間はパネルの下に隠れる (シーンの慣例どおり)。
///
/// 指令経路は JointStateSub.Callback を鏡写しにする:
///   - ServoJointModel がある関節 → servo.SetCommand(rad, 0)
///   - それ以外 → xDrive.target へ書く (回転関節は rad→deg 変換、直動は m のまま)
/// つまり手動スライダと ROS の /joint_states 指令は同じ書き込み先を取り合い、
/// 後から書いたほうが勝つ。ドラッグしていない間はスライダが関節の現在値に
/// 追従するので、ROS からの指令も見える。
/// </remarks>
public class EntityListPanel : MonoBehaviour
{
    const float ListPollInterval = 1f;    // エンティティ一覧のポーリング周期 [s]
    const float ValueSyncInterval = 0.1f; // スライダ表示の追従周期 [s]

    // 右下ボタン列 (AddButton -125 / SaveButton -75 / LoadButton -25、いずれも
    // 40x40・間隔 50・右端の隙間 5) と揃えるための寸法。
    const float ToggleSize = 40f;
    const float ToggleRowNextEdge = 155f; // 列の次の空きスロットの右端 (AddButton 左端 -145 の 10px 左)

    // 右側サイドバー (SideBarPanel) が見つからないときの保険値。
    // 値はシーン YAML の SideBarPanel / SidebarController と同じ。
    const float FallbackPanelWidth = 300f;
    const float FallbackSlideDuration = 0.3f;
    static readonly Color FallbackPanelColor = new Color(1f, 1f, 1f, 0.392f);

    static readonly Color TextColor = new Color(0.196f, 0.196f, 0.196f, 1f);
    static readonly Color HeaderColor = Color.black; // サイドバー見出し (Environment Settings 等) と同じ
    static readonly Color SelectedColor = Color.cyan; // ObjectSpawner の選択色と同じ
    static readonly Color SliderBgColor = new Color(0.78f, 0.78f, 0.78f, 1f);
    static readonly Color SliderFillColor = new Color(0.25f, 0.75f, 0.85f, 1f);

    SimulationControl m_Control;
    GameObject m_ListItemPrefab;
    Sprite m_UiSprite;
    Sprite m_PanelSprite;     // サイドバーの背景 (組み込み Background) を実行時にコピー
    Color m_PanelColor = FallbackPanelColor;

    GameObject m_Panel;
    RectTransform m_PanelRt;
    RectTransform m_EntityContent;
    RectTransform m_JointContent;
    TMP_Text m_JointHeader;
    GameObject m_EntityEmptyHint; // 一覧が空のときの案内行
    GameObject m_JointHint;       // 未選択のときの案内行

    // サイドバーと同じ開閉アニメーション (anchoredPosition の Lerp)
    Vector2 m_HiddenPos;
    Vector2 m_ShownPos;
    bool m_IsOpen;
    float m_SlideDuration = FallbackSlideDuration;
    Coroutine m_SlideRoutine;

    // 使い回しバッファ (ポーリングでの毎回アロケーション回避)
    readonly List<GameObject> m_EntityBuffer = new List<GameObject>();
    readonly List<GameObject> m_ShownEntities = new List<GameObject>();
    readonly List<Button> m_EntityButtons = new List<Button>();
    readonly List<SimulationControl.GuiJointInfo> m_JointBuffer =
        new List<SimulationControl.GuiJointInfo>();

    GameObject m_SelectedEntity;
    readonly List<JointRow> m_JointRows = new List<JointRow>();
    readonly List<SimulationControl.GuiSensorInfo> m_SensorBuffer =
        new List<SimulationControl.GuiSensorInfo>();
    readonly List<SensorRow> m_SensorRows = new List<SensorRow>();
    readonly List<GameObject> m_SensorSectionRows = new List<GameObject>(); // 見出し・説明行

    float m_NextListPoll;
    float m_NextValueSync;

    class JointRow
    {
        public SimulationControl.GuiJointInfo joint;
        public GameObject root;
        public Slider slider;
        public TMP_Text label;
        public SliderDragWatcher drag;
        public float minDisplay;
        public float maxDisplay;
        public float lastLabelValue = float.NaN;
    }

    class SensorRow
    {
        public SimulationControl.GuiSensorInfo info;
        public GameObject root;
        public Image toggleImage;
        public TMP_Text toggleLabel;
        public Component visualizer; // 有効中の可視化コンポーネント (無効なら null)
        public GameObject previewRow; // 画像系のみ: パネル内のプレビュー行

        public bool IsImageKind =>
            info.kind == SimulationControl.GuiSensorVizKind.ImageTexture0
            || info.kind == SimulationControl.GuiSensorVizKind.ImageTexture1;
    }

    void Start()
    {
        m_Control = GetComponent<SimulationControl>();
        if (m_Control == null)
        {
            Debug.LogError("[EntityListPanel] SimulationControl not found on the same GameObject.");
            enabled = false;
            return;
        }

        // 景観リストと同じ行ボタン。Resources 配下なのでランタイムでも読める。
        m_ListItemPrefab = Resources.Load<GameObject>("Prefabs/ListItemPrehab");
        if (m_ListItemPrefab != null)
        {
            Image prefabImage = m_ListItemPrefab.GetComponent<Image>();
            if (prefabImage != null)
            {
                m_UiSprite = prefabImage.sprite; // 組み込み UISprite。パネルやスライダにも使う
            }
        }

        Canvas canvas = FindTargetCanvas();
        BuildUi(canvas);

        // デバッグ/ヘッドレステスト用: SIM_AUTO_SENSOR_VIZ=N (N>=1) で起動すると、
        // 最初にスポーンされたエンティティを自動選択して全センサ可視化を ON にし、
        // OFF→ON のトグルを N 回繰り返す (最後は ON のまま)。GUI クリック無しで
        // 可視化経路 (シェーダ・Visualizer の実行時取り付け・解除) を通せるように
        // するための入り口で、通常運用では未設定のまま何もしない。
        string autoViz = System.Environment.GetEnvironmentVariable("SIM_AUTO_SENSOR_VIZ");
        if (!string.IsNullOrEmpty(autoViz) && int.TryParse(autoViz, out int cycles) && cycles > 0)
        {
            StartCoroutine(AutoEnableSensorVisualization(cycles));
        }
    }

    IEnumerator AutoEnableSensorVisualization(int cycles)
    {
        // パネルを開いてポーリングを動かし、エンティティが現れたら選択する。
        if (!m_IsOpen)
        {
            TogglePanel();
        }
        while (m_EntityButtons.Count == 0)
        {
            yield return null;
        }
        m_EntityButtons[0].onClick.Invoke();
        yield return null;
        // ON→OFF→ON を繰り返し、有効化だけでなく解除経路 (可視化コンポーネントの
        // 破棄とセンサ更新イベントの購読解除) も通す。最後は ON のままにする。
        Debug.Log($"[EntityListPanel] auto viz: enabling {m_SensorRows.Count} sensor visualizations");
        ToggleAllSensorRows(wantOn: true);
        for (int i = 0; i < cycles; i++)
        {
            yield return new WaitForSecondsRealtime(5f);
            Debug.Log($"[EntityListPanel] auto viz: cycle {i + 1}/{cycles} disabling all");
            ToggleAllSensorRows(wantOn: false);
            yield return new WaitForSecondsRealtime(3f);
            Debug.Log($"[EntityListPanel] auto viz: cycle {i + 1}/{cycles} re-enabling all");
            ToggleAllSensorRows(wantOn: true);
        }
        Debug.Log("[EntityListPanel] auto viz: done");
    }

    void ToggleAllSensorRows(bool wantOn)
    {
        foreach (SensorRow row in m_SensorRows)
        {
            if ((row.visualizer != null) != wantOn)
            {
                ToggleSensorRow(row);
            }
        }
    }

    Canvas FindTargetCanvas()
    {
        foreach (Canvas candidate in FindObjectsByType<Canvas>(FindObjectsSortMode.None))
        {
            if (candidate.isRootCanvas && candidate.renderMode == RenderMode.ScreenSpaceOverlay)
            {
                return candidate;
            }
        }
        // 保険: シーンに Canvas が無いときだけ最小構成で作る。
        var go = new GameObject("EntityPanelCanvas", typeof(Canvas), typeof(CanvasScaler), typeof(GraphicRaycaster));
        Canvas created = go.GetComponent<Canvas>();
        created.renderMode = RenderMode.ScreenSpaceOverlay;
        return created;
    }

    // ------------------------------------------------------------------ UI 構築

    void BuildUi(Canvas canvas)
    {
        RectTransform canvasRt = canvas.GetComponent<RectTransform>();

        // 右側サイドバー (設定パネル) の見た目と開閉アニメーションを鏡写しにする。
        // 幅・背景・スライド時間は実行時に SidebarController からコピーする。
        float width = FallbackPanelWidth;
        m_PanelSprite = m_UiSprite;
        RectTransform styleSource = FindSidebarStyleSource(out SidebarController sidebarController);
        if (styleSource != null)
        {
            width = styleSource.rect.width;
            Image sidebarImage = styleSource.GetComponent<Image>();
            if (sidebarImage != null)
            {
                m_PanelSprite = sidebarImage.sprite;
                m_PanelColor = sidebarImage.color;
            }
            if (sidebarController.duration > 0f)
            {
                m_SlideDuration = sidebarController.duration;
            }
        }

        // 開くボタン: 右下のボタン列 (Add/Save/Load) の続き、次の空きスロットへ。
        // AddButton / HamburgerButton と同じくパネルより先に生成する = 開いている
        // 間はパネルの下に隠れ、閉じる操作はパネル内の CloseButton が受け持つ。
        GameObject toggle = CreateToggleButton(canvasRt);
        RectTransform toggleRt = toggle.GetComponent<RectTransform>();
        toggleRt.anchorMin = new Vector2(1f, 0f);
        toggleRt.anchorMax = new Vector2(1f, 0f);
        toggleRt.pivot = new Vector2(0.5f, 0.5f);
        // AddButton の 10px 左 (右端 -155) に右端を揃える。文字ボタンへの代替時
        // (幅 100) も右端が揃うよう幅から中心を出す。40x40 なら (-175, 25)。
        toggleRt.anchoredPosition = new Vector2(-(ToggleRowNextEdge + toggleRt.sizeDelta.x / 2f), 25f);
        toggle.GetComponent<Button>().onClick.AddListener(TogglePanel);

        // パネル本体: SideBarPanel と同じ右端・縦いっぱい。表示/非表示の
        // anchoredPosition も SidebarController.Start の規約そのまま
        // (表示は端にピタリ = -幅/2、非表示はアンカー右端外 = +幅)。
        // ランタイムで最後に生成される = 既存サイドバーより手前に描かれる。
        m_Panel = CreateUiObject("EntityPanel", canvasRt);
        m_PanelRt = m_Panel.GetComponent<RectTransform>();
        m_PanelRt.anchorMin = new Vector2(1f, 0f);
        m_PanelRt.anchorMax = new Vector2(1f, 1f);
        m_PanelRt.pivot = new Vector2(0.5f, 0.5f);
        m_PanelRt.sizeDelta = new Vector2(width, 0f);
        m_ShownPos = new Vector2(-width / 2f, 0f);
        m_HiddenPos = new Vector2(width, 0f);
        m_PanelRt.anchoredPosition = m_HiddenPos; // 初期状態は隠す (SidebarController.Start と同じ)
        Image panelImage = m_Panel.AddComponent<Image>();
        panelImage.sprite = m_PanelSprite;
        panelImage.type = Image.Type.Sliced;
        panelImage.color = m_PanelColor;

        // 見出しと説明は右上の CloseButton (上から 45px・右から 45px を占有) の
        // 下へ固定 px で置く。割合指定だとウィンドウが低いときにボタンへ潜るため。
        TMP_Text title = CreateLabel("Title", m_PanelRt, "Entities",
            new Vector2(0.04f, 1f), new Vector2(0.96f, 1f), 24f, HeaderColor);
        title.rectTransform.offsetMin = new Vector2(0f, -88f);
        title.rectTransform.offsetMax = new Vector2(0f, -50f);
        TMP_Text caption = CreateLabel("TitleCaption", m_PanelRt,
            "Spawned robots/objects (services & GUI)",
            new Vector2(0.04f, 1f), new Vector2(0.96f, 1f), 14f, TextColor);
        caption.rectTransform.offsetMin = new Vector2(0f, -118f);
        caption.rectTransform.offsetMax = new Vector2(0f, -88f);

        m_EntityContent = CreateScrollList("EntityList", m_PanelRt,
            new Vector2(0.02f, 0.62f), new Vector2(0.98f, 1f));
        // リスト上端もヘッダの下 (上から 122px) に固定する。
        // m_EntityContent は ScrollRect の Content。ScrollRect のルートは 2 つ上。
        ((RectTransform)m_EntityContent.parent.parent).offsetMax = new Vector2(0f, -122f);

        m_JointHeader = CreateLabel("JointHeader", m_PanelRt, "Joints",
            new Vector2(0.04f, 0.575f), new Vector2(0.96f, 0.615f), 20f, HeaderColor);
        CreateLabel("JointCaption", m_PanelRt, "Drag to command; tracks ROS otherwise",
            new Vector2(0.04f, 0.545f), new Vector2(0.96f, 0.575f), 14f, TextColor);

        m_JointContent = CreateScrollList("JointList", m_PanelRt,
            new Vector2(0.02f, 0.01f), new Vector2(0.98f, 0.54f));

        // 初期状態の案内行 (一覧は空・未選択で始まる)。
        m_EntityEmptyHint = CreateHintRow(m_EntityContent, "No spawned entities");
        m_JointHint = CreateHintRow(m_JointContent, "Select an entity to inspect its joints");

        // 閉じるボタン: 既存パネルの CloseButton と同じ位置・大きさ・作法。
        CreateCloseButton(m_PanelRt, styleSource);
    }

    /// <summary>
    /// 既存サイドバーと同じ閉じるボタン (パネル右上 (-25,-25)、40x40)。アイコンは
    /// 既存サイドバー内の CloseButton (Assets/Scenes/CloseButton.png、Resources 外
    /// なので実行時ロード不可) からコピーし、見つからなければ "X" の文字にする。
    /// 既存の CloseButton が開くボタンと同じ ToggleSidebar を呼ぶのに合わせて、
    /// こちらも TogglePanel を呼ぶ。
    /// </summary>
    void CreateCloseButton(RectTransform panelRt, RectTransform sidebarRt)
    {
        Sprite icon = null;
        if (sidebarRt != null)
        {
            Transform existing = sidebarRt.Find("CloseButton");
            if (existing != null)
            {
                Image existingImage = existing.GetComponent<Image>();
                if (existingImage != null)
                {
                    icon = existingImage.sprite;
                }
            }
        }

        GameObject close = CreateUiObject("CloseButton", panelRt);
        RectTransform closeRt = close.GetComponent<RectTransform>();
        closeRt.anchorMin = new Vector2(1f, 1f);
        closeRt.anchorMax = new Vector2(1f, 1f);
        closeRt.pivot = new Vector2(0.5f, 0.5f);
        closeRt.sizeDelta = new Vector2(ToggleSize, ToggleSize);
        closeRt.anchoredPosition = new Vector2(-25f, -25f);

        Image image = close.AddComponent<Image>();
        Button button = close.AddComponent<Button>();
        button.targetGraphic = image;
        if (icon != null)
        {
            image.sprite = icon;
            image.type = Image.Type.Simple;
            image.color = Color.white;
        }
        else
        {
            // アイコンが取れないときは既存ボタンの TMP 子と同じ体裁の "X" で代用。
            image.sprite = m_UiSprite;
            image.type = Image.Type.Sliced;
            image.color = Color.white;
            GameObject textGo = CreateUiObject("Text (TMP)", close.transform);
            RectTransform textRt = textGo.GetComponent<RectTransform>();
            textRt.anchorMin = Vector2.zero;
            textRt.anchorMax = Vector2.one;
            textRt.offsetMin = Vector2.zero;
            textRt.offsetMax = Vector2.zero;
            TextMeshProUGUI tmp = textGo.AddComponent<TextMeshProUGUI>();
            tmp.text = "X";
            tmp.fontSize = 24f;
            tmp.color = TextColor;
            tmp.alignment = TextAlignmentOptions.Center;
        }
        button.onClick.AddListener(TogglePanel);
    }

    /// <summary>リスト内の案内行 (空一覧・未選択のときのプレースホルダ)。</summary>
    GameObject CreateHintRow(RectTransform parent, string text)
    {
        GameObject go = CreateUiObject("Hint", parent);
        go.GetComponent<RectTransform>().sizeDelta = new Vector2(0f, 30f);
        TextMeshProUGUI tmp = go.AddComponent<TextMeshProUGUI>();
        tmp.text = text;
        tmp.fontSize = 14f;
        tmp.color = TextColor;
        tmp.alignment = TextAlignmentOptions.Center;
        return go;
    }

    /// <summary>
    /// 見た目の複製元にする既存サイドバーを探す。複数あるとき (設定 300 / 景観 350)
    /// は幅が最小のもの = 設定サイドバー (SideBarPanel) を選ぶ。
    /// </summary>
    static RectTransform FindSidebarStyleSource(out SidebarController controller)
    {
        controller = null;
        RectTransform best = null;
        foreach (SidebarController candidate in FindObjectsByType<SidebarController>(
            FindObjectsInactive.Include, FindObjectsSortMode.None))
        {
            if (candidate.sidebar == null)
            {
                continue;
            }
            if (best == null || candidate.sidebar.rect.width < best.rect.width)
            {
                best = candidate.sidebar;
                controller = candidate;
            }
        }
        return best;
    }

    /// <summary>
    /// 右下ボタン列 (Add/Save/Load) と同じ作りの開閉ボタン (Image + Button、40x40)。アイコンは
    /// Resources/EntityButton.png。Sprite として読めなければ Texture2D から作り、
    /// それも無ければ文字 "Entities" のボタンにする。
    /// </summary>
    GameObject CreateToggleButton(RectTransform parent)
    {
        Sprite icon = Resources.Load<Sprite>("EntityButton");
        if (icon == null)
        {
            // インポータが Sprite になっていない (Texture2D 扱いの) とき用の保険。
            Texture2D tex = Resources.Load<Texture2D>("EntityButton");
            if (tex != null)
            {
                icon = Sprite.Create(tex, new Rect(0f, 0f, tex.width, tex.height),
                    new Vector2(0.5f, 0.5f));
            }
        }

        GameObject toggle;
        if (icon != null)
        {
            // AddButton / SaveButton / LoadButton と同じ構成: Image (Simple, 白) + Button。
            toggle = CreateUiObject("EntityPanelToggle", parent);
            toggle.GetComponent<RectTransform>().sizeDelta = new Vector2(ToggleSize, ToggleSize);
            Image image = toggle.AddComponent<Image>();
            image.sprite = icon;
            image.type = Image.Type.Simple;
            image.color = Color.white;
            Button button = toggle.AddComponent<Button>();
            button.targetGraphic = image;
        }
        else
        {
            // アイコンが読めないときは文字入りの行ボタンで代用する。
            toggle = CreateItemButton("Entities", parent);
            toggle.name = "EntityPanelToggle";
            toggle.GetComponent<RectTransform>().sizeDelta = new Vector2(100f, ToggleSize);
            Image image = toggle.GetComponent<Image>();
            if (image != null)
            {
                image.color = Color.white; // 隣のボタンと同じ白 (プレハブは薄ピンク)
            }
        }
        return toggle;
    }

    void TogglePanel()
    {
        m_IsOpen = !m_IsOpen;
        if (m_IsOpen)
        {
            m_NextListPoll = 0f; // 開いた瞬間に一覧を作り直す
            m_NextValueSync = 0f;
        }
        // SidebarController.ToggleSidebar と同じ: 進行中の開閉は打ち切って逆向きに。
        if (m_SlideRoutine != null)
        {
            StopCoroutine(m_SlideRoutine);
        }
        m_SlideRoutine = StartCoroutine(SlidePanel(m_IsOpen ? m_ShownPos : m_HiddenPos));
    }

    /// <summary>SidebarController.AnimateSidebar と同じ補間 (Lerp + unscaled 時間)。</summary>
    IEnumerator SlidePanel(Vector2 target)
    {
        Vector2 start = m_PanelRt.anchoredPosition;
        float elapsed = 0f;
        while (elapsed < m_SlideDuration)
        {
            elapsed += Time.unscaledDeltaTime;
            m_PanelRt.anchoredPosition = Vector2.Lerp(start, target, elapsed / m_SlideDuration);
            yield return null;
        }
        m_PanelRt.anchoredPosition = target;
    }

    GameObject CreateUiObject(string name, Transform parent)
    {
        var go = new GameObject(name, typeof(RectTransform));
        go.layer = 5; // UI
        go.transform.SetParent(parent, false);
        return go;
    }

    /// <summary>
    /// 景観リストと同じ見た目の行ボタン。プレハブがあればそれを複製し、
    /// 無ければ同じ構成 (Image + Button + TMP 子) をコードで組む。
    /// </summary>
    GameObject CreateItemButton(string text, Transform parent)
    {
        GameObject item;
        if (m_ListItemPrefab != null)
        {
            item = Instantiate(m_ListItemPrefab, parent);
        }
        else
        {
            item = CreateUiObject("ListItem", parent);
            item.GetComponent<RectTransform>().sizeDelta = new Vector2(240f, 30f);
            Image image = item.AddComponent<Image>();
            image.sprite = m_UiSprite;
            image.type = Image.Type.Sliced;
            Button button = item.AddComponent<Button>();
            button.targetGraphic = image;
            GameObject textGo = CreateUiObject("Text (TMP)", item.transform);
            RectTransform textRt = textGo.GetComponent<RectTransform>();
            textRt.anchorMin = Vector2.zero;
            textRt.anchorMax = Vector2.one;
            textRt.offsetMin = Vector2.zero;
            textRt.offsetMax = Vector2.zero;
            TextMeshProUGUI tmp = textGo.AddComponent<TextMeshProUGUI>();
            tmp.fontSize = 18f;
            tmp.color = TextColor;
            tmp.alignment = TextAlignmentOptions.Center;
        }

        TMP_Text label = item.GetComponentInChildren<TMP_Text>();
        if (label != null)
        {
            label.text = text;
        }
        return item;
    }

    TMP_Text CreateLabel(string name, RectTransform parent, string text,
        Vector2 anchorMin, Vector2 anchorMax, float fontSize, Color color)
    {
        GameObject go = CreateUiObject(name, parent);
        RectTransform rt = go.GetComponent<RectTransform>();
        rt.anchorMin = anchorMin;
        rt.anchorMax = anchorMax;
        rt.offsetMin = Vector2.zero;
        rt.offsetMax = Vector2.zero;
        TextMeshProUGUI tmp = go.AddComponent<TextMeshProUGUI>();
        tmp.text = text;
        tmp.fontSize = fontSize;
        tmp.color = color;
        tmp.alignment = TextAlignmentOptions.MidlineLeft;
        tmp.textWrappingMode = TextWrappingModes.NoWrap;
        tmp.overflowMode = TextOverflowModes.Ellipsis;
        return tmp;
    }

    /// <summary>縦スクロールするリスト領域を作り、行を入れる Content を返す。</summary>
    RectTransform CreateScrollList(string name, RectTransform parent,
        Vector2 anchorMin, Vector2 anchorMax)
    {
        GameObject scrollGo = CreateUiObject(name, parent);
        RectTransform scrollRt = scrollGo.GetComponent<RectTransform>();
        scrollRt.anchorMin = anchorMin;
        scrollRt.anchorMax = anchorMax;
        scrollRt.offsetMin = Vector2.zero;
        scrollRt.offsetMax = Vector2.zero;
        // サイドバー内の Scroll View と同じ背景 (Background スプライト、白 α0.392)。
        Image scrollBg = scrollGo.AddComponent<Image>();
        scrollBg.sprite = m_PanelSprite;
        scrollBg.type = Image.Type.Sliced;
        scrollBg.color = m_PanelColor;
        ScrollRect scroll = scrollGo.AddComponent<ScrollRect>();

        GameObject viewportGo = CreateUiObject("Viewport", scrollRt);
        RectTransform viewportRt = viewportGo.GetComponent<RectTransform>();
        viewportRt.anchorMin = Vector2.zero;
        viewportRt.anchorMax = Vector2.one;
        viewportRt.offsetMin = Vector2.zero;
        viewportRt.offsetMax = Vector2.zero;
        viewportRt.pivot = new Vector2(0.5f, 1f);
        viewportGo.AddComponent<RectMask2D>();

        GameObject contentGo = CreateUiObject("Content", viewportRt);
        RectTransform contentRt = contentGo.GetComponent<RectTransform>();
        contentRt.anchorMin = new Vector2(0f, 1f);
        contentRt.anchorMax = new Vector2(1f, 1f);
        contentRt.pivot = new Vector2(0.5f, 1f);
        contentRt.sizeDelta = Vector2.zero;
        VerticalLayoutGroup layout = contentGo.AddComponent<VerticalLayoutGroup>();
        layout.padding = new RectOffset(6, 6, 6, 6);
        layout.spacing = 4f;
        layout.childAlignment = TextAnchor.UpperCenter;
        layout.childControlWidth = true;
        layout.childControlHeight = false;
        layout.childForceExpandWidth = true;
        layout.childForceExpandHeight = false;
        ContentSizeFitter fitter = contentGo.AddComponent<ContentSizeFitter>();
        fitter.verticalFit = ContentSizeFitter.FitMode.PreferredSize;

        scroll.viewport = viewportRt;
        scroll.content = contentRt;
        scroll.horizontal = false;
        scroll.vertical = true;
        scroll.movementType = ScrollRect.MovementType.Clamped;
        scroll.scrollSensitivity = 20f;
        return contentRt;
    }

    /// <summary>Background / Fill / Handle を持つ標準構成の Slider を組む。</summary>
    Slider CreateSlider(RectTransform parent)
    {
        GameObject sliderGo = CreateUiObject("Slider", parent);

        GameObject bgGo = CreateUiObject("Background", sliderGo.transform);
        RectTransform bgRt = bgGo.GetComponent<RectTransform>();
        bgRt.anchorMin = new Vector2(0f, 0.3f);
        bgRt.anchorMax = new Vector2(1f, 0.7f);
        bgRt.offsetMin = Vector2.zero;
        bgRt.offsetMax = Vector2.zero;
        Image bgImage = bgGo.AddComponent<Image>();
        bgImage.sprite = m_UiSprite;
        bgImage.type = Image.Type.Sliced;
        bgImage.color = SliderBgColor;

        GameObject fillAreaGo = CreateUiObject("Fill Area", sliderGo.transform);
        RectTransform fillAreaRt = fillAreaGo.GetComponent<RectTransform>();
        fillAreaRt.anchorMin = new Vector2(0f, 0.3f);
        fillAreaRt.anchorMax = new Vector2(1f, 0.7f);
        fillAreaRt.offsetMin = new Vector2(5f, 0f);
        fillAreaRt.offsetMax = new Vector2(-15f, 0f);

        GameObject fillGo = CreateUiObject("Fill", fillAreaRt);
        RectTransform fillRt = fillGo.GetComponent<RectTransform>();
        fillRt.sizeDelta = new Vector2(10f, 0f);
        Image fillImage = fillGo.AddComponent<Image>();
        fillImage.sprite = m_UiSprite;
        fillImage.type = Image.Type.Sliced;
        fillImage.color = SliderFillColor;

        GameObject handleAreaGo = CreateUiObject("Handle Slide Area", sliderGo.transform);
        RectTransform handleAreaRt = handleAreaGo.GetComponent<RectTransform>();
        handleAreaRt.anchorMin = Vector2.zero;
        handleAreaRt.anchorMax = Vector2.one;
        handleAreaRt.offsetMin = new Vector2(10f, 0f);
        handleAreaRt.offsetMax = new Vector2(-10f, 0f);

        GameObject handleGo = CreateUiObject("Handle", handleAreaRt);
        RectTransform handleRt = handleGo.GetComponent<RectTransform>();
        handleRt.sizeDelta = new Vector2(20f, 0f);
        Image handleImage = handleGo.AddComponent<Image>();
        handleImage.sprite = m_UiSprite;
        handleImage.type = Image.Type.Sliced;
        handleImage.color = Color.white;

        Slider slider = sliderGo.AddComponent<Slider>();
        slider.fillRect = fillRt;
        slider.handleRect = handleRt;
        slider.targetGraphic = handleImage;
        slider.direction = Slider.Direction.LeftToRight;
        return slider;
    }

    // -------------------------------------------------------------- 更新ループ

    void Update()
    {
        // 閉じている間 (スライドアウト中を含む) はポーリングも追従も止める。
        if (m_Panel == null || !m_IsOpen)
        {
            return;
        }

        float now = Time.unscaledTime;

        if (now >= m_NextListPoll)
        {
            m_NextListPoll = now + ListPollInterval;
            RefreshEntityListIfChanged();
        }

        // 選択中のエンティティがデスポーンされたら関節表示を畳む。
        if (m_SelectedEntity == null && m_JointRows.Count > 0)
        {
            ClearSelection();
        }

        if (now >= m_NextValueSync)
        {
            m_NextValueSync = now + ValueSyncInterval;
            SyncJointRows();
        }
    }

    void RefreshEntityListIfChanged()
    {
        m_Control.GetEntitiesSnapshot(m_EntityBuffer);

        bool changed = m_EntityBuffer.Count != m_ShownEntities.Count;
        if (!changed)
        {
            for (int i = 0; i < m_EntityBuffer.Count; i++)
            {
                if (!ReferenceEquals(m_EntityBuffer[i], m_ShownEntities[i]))
                {
                    changed = true;
                    break;
                }
            }
        }
        if (!changed)
        {
            return;
        }

        foreach (Button button in m_EntityButtons)
        {
            if (button != null)
            {
                Destroy(button.gameObject);
            }
        }
        m_EntityButtons.Clear();
        m_ShownEntities.Clear();
        if (m_EntityEmptyHint != null)
        {
            Destroy(m_EntityEmptyHint);
            m_EntityEmptyHint = null;
        }
        if (m_EntityBuffer.Count == 0)
        {
            m_EntityEmptyHint = CreateHintRow(m_EntityContent, "No spawned entities");
        }

        bool selectedStillExists = false;
        foreach (GameObject entity in m_EntityBuffer)
        {
            m_ShownEntities.Add(entity);
            GameObject item = CreateItemButton(entity.name, m_EntityContent);
            Button button = item.GetComponent<Button>();
            m_EntityButtons.Add(button);
            GameObject captured = entity;
            Button capturedButton = button;
            button.onClick.AddListener(() => OnSelectEntity(captured, capturedButton));
            if (ReferenceEquals(entity, m_SelectedEntity))
            {
                selectedStillExists = true;
                button.image.color = SelectedColor;
            }
        }

        if (m_SelectedEntity != null && !selectedStillExists)
        {
            ClearSelection();
        }
    }

    void OnSelectEntity(GameObject entity, Button button)
    {
        if (entity == null)
        {
            // 破棄済みの行を押した (次のポーリングで行ごと消える)。
            ClearSelection();
            return;
        }

        foreach (Button other in m_EntityButtons)
        {
            if (other != null)
            {
                other.image.color = Color.white;
            }
        }

        if (ReferenceEquals(entity, m_SelectedEntity))
        {
            ClearSelection(); // 再クリックで選択解除 (景観リストと同じ操作感)
            return;
        }

        m_SelectedEntity = entity;
        if (button != null)
        {
            button.image.color = SelectedColor;
        }
        BuildJointRows(entity);
    }

    void ClearSelection()
    {
        m_SelectedEntity = null;
        foreach (JointRow row in m_JointRows)
        {
            if (row.root != null)
            {
                Destroy(row.root);
            }
        }
        m_JointRows.Clear();
        ClearSensorRows();
        if (m_JointHeader != null)
        {
            m_JointHeader.text = "Joints";
        }
        // 未選択に戻ったら案内行を出し直す。
        if (m_JointHint == null && m_JointContent != null)
        {
            m_JointHint = CreateHintRow(m_JointContent, "Select an entity to inspect its joints");
        }
        foreach (Button button in m_EntityButtons)
        {
            if (button != null)
            {
                button.image.color = Color.white;
            }
        }
    }

    // ---------------------------------------------------------- 関節スライダ

    void BuildJointRows(GameObject entity)
    {
        foreach (JointRow oldRow in m_JointRows)
        {
            if (oldRow.root != null)
            {
                Destroy(oldRow.root);
            }
        }
        m_JointRows.Clear();
        if (m_JointHint != null)
        {
            Destroy(m_JointHint);
            m_JointHint = null;
        }

        m_JointHeader.text = "Joints: " + entity.name;
        SimulationControl.GetMovableJoints(entity, m_JointBuffer);

        if (m_JointBuffer.Count == 0)
        {
            m_JointRows.Add(new JointRow
            {
                root = CreateHintRow(m_JointContent, "(no movable joints)"),
            });
        }
        else
        {
            foreach (SimulationControl.GuiJointInfo joint in m_JointBuffer)
            {
                m_JointRows.Add(CreateJointRow(joint));
            }
        }

        BuildSensorRows(entity);
    }

    // ------------------------------------------------------------ センサ可視化

    /// <summary>
    /// 関節リストの続きに Sensors 節を出す。行ごとの On/Off で UnitySensors の
    /// 可視化 (点群は 3D シーンへの重畳、カメラは行の下のプレビュー画像) を
    /// 切り替える。点群の可視化は選択を外しても付けたままにして、パネルを
    /// 閉じても見え続けるようにする (再選択でトグル状態を復元する)。
    /// 画像プレビューはパネル内の行が表示先なので、行が消えるときに一緒に外す。
    /// </summary>
    void BuildSensorRows(GameObject entity)
    {
        ClearSensorRows();
        SimulationControl.GetVisualizableSensors(entity, m_SensorBuffer);
        if (m_SensorBuffer.Count == 0)
        {
            return;
        }

        m_SensorSectionRows.Add(CreateSectionHeaderRow("Sensors"));
        m_SensorSectionRows.Add(CreateHintRow(m_JointContent,
            "Toggle 3D/point-cloud & camera previews"));

        foreach (SimulationControl.GuiSensorInfo info in m_SensorBuffer)
        {
            m_SensorRows.Add(CreateSensorRow(info));
        }
    }

    void ClearSensorRows()
    {
        foreach (SensorRow row in m_SensorRows)
        {
            // 画像系はプレビュー行 (表示先) ごと可視化を外す。点群系は残す。
            if (row.IsImageKind && row.visualizer != null)
            {
                Destroy(row.visualizer);
            }
            if (row.previewRow != null)
            {
                Destroy(row.previewRow);
            }
            if (row.root != null)
            {
                Destroy(row.root);
            }
        }
        m_SensorRows.Clear();
        foreach (GameObject sectionRow in m_SensorSectionRows)
        {
            if (sectionRow != null)
            {
                Destroy(sectionRow);
            }
        }
        m_SensorSectionRows.Clear();
    }

    /// <summary>リスト内のインライン見出し行 (パネル見出しと同じ黒・左寄せ)。</summary>
    GameObject CreateSectionHeaderRow(string text)
    {
        GameObject go = CreateUiObject("SectionHeader", m_JointContent);
        go.GetComponent<RectTransform>().sizeDelta = new Vector2(0f, 30f);
        TextMeshProUGUI tmp = go.AddComponent<TextMeshProUGUI>();
        tmp.text = text;
        tmp.fontSize = 20f;
        tmp.color = HeaderColor;
        tmp.alignment = TextAlignmentOptions.MidlineLeft;
        return go;
    }

    SensorRow CreateSensorRow(SimulationControl.GuiSensorInfo info)
    {
        GameObject rowGo = CreateUiObject(info.label, m_JointContent);
        rowGo.GetComponent<RectTransform>().sizeDelta = new Vector2(0f, 32f);

        GameObject labelGo = CreateUiObject("Label", rowGo.transform);
        RectTransform labelRt = labelGo.GetComponent<RectTransform>();
        labelRt.anchorMin = new Vector2(0f, 0f);
        labelRt.anchorMax = new Vector2(0.68f, 1f);
        labelRt.offsetMin = new Vector2(4f, 0f);
        labelRt.offsetMax = new Vector2(-2f, 0f);
        TextMeshProUGUI label = labelGo.AddComponent<TextMeshProUGUI>();
        label.text = info.label;
        label.fontSize = 14f;
        label.color = TextColor;
        label.alignment = TextAlignmentOptions.MidlineLeft;
        label.textWrappingMode = TextWrappingModes.NoWrap;
        label.overflowMode = TextOverflowModes.Ellipsis;

        // 行ボタンと同じ部材 (UISprite + Button + TMP 子) の小さな On/Off ボタン。
        GameObject buttonGo = CreateUiObject("Toggle", rowGo.transform);
        RectTransform buttonRt = buttonGo.GetComponent<RectTransform>();
        buttonRt.anchorMin = new Vector2(0.7f, 0.08f);
        buttonRt.anchorMax = new Vector2(1f, 0.92f);
        buttonRt.offsetMin = Vector2.zero;
        buttonRt.offsetMax = new Vector2(-4f, 0f);
        Image buttonImage = buttonGo.AddComponent<Image>();
        buttonImage.sprite = m_UiSprite;
        buttonImage.type = Image.Type.Sliced;
        buttonImage.color = Color.white;
        Button button = buttonGo.AddComponent<Button>();
        button.targetGraphic = buttonImage;
        GameObject buttonTextGo = CreateUiObject("Text (TMP)", buttonGo.transform);
        RectTransform buttonTextRt = buttonTextGo.GetComponent<RectTransform>();
        buttonTextRt.anchorMin = Vector2.zero;
        buttonTextRt.anchorMax = Vector2.one;
        buttonTextRt.offsetMin = Vector2.zero;
        buttonTextRt.offsetMax = Vector2.zero;
        TextMeshProUGUI buttonText = buttonTextGo.AddComponent<TextMeshProUGUI>();
        buttonText.fontSize = 14f;
        buttonText.color = TextColor;
        buttonText.alignment = TextAlignmentOptions.Center;

        var row = new SensorRow
        {
            info = info,
            root = rowGo,
            toggleImage = buttonImage,
            toggleLabel = buttonText,
        };

        // 点群系は前回有効にした可視化が生きていればトグル状態を復元する。
        if (!row.IsImageKind)
        {
            row.visualizer = SensorVisualization.FindAttachedPointCloud(info);
        }
        UpdateSensorRowVisual(row);

        button.onClick.AddListener(() => ToggleSensorRow(row));
        return row;
    }

    void ToggleSensorRow(SensorRow row)
    {
        if (row.info.sensor == null)
        {
            return; // エンティティ破棄済み。次の Update の選択チェックで畳まれる
        }

        if (row.visualizer != null)
        {
            if (row.IsImageKind)
            {
                Destroy(row.visualizer);
                if (row.previewRow != null)
                {
                    Destroy(row.previewRow);
                    row.previewRow = null;
                }
            }
            else
            {
                SensorVisualization.DetachPointCloud(row.info, row.visualizer);
            }
            row.visualizer = null;
        }
        else if (row.IsImageKind)
        {
            row.visualizer = AttachImagePreview(row);
        }
        else
        {
            row.visualizer = SensorVisualization.AttachPointCloud(row.info);
        }
        UpdateSensorRowVisual(row);
    }

    void UpdateSensorRowVisual(SensorRow row)
    {
        bool on = row.visualizer != null;
        row.toggleLabel.text = on ? "On" : "Off";
        row.toggleImage.color = on ? SelectedColor : Color.white;
    }

    /// <summary>
    /// センサ行の直後にプレビュー行 (RawImage) を差し込み、UnitySensors の
    /// TextureVisualizer をセンサへ取り付けて表示先にする。行の高さはカメラの
    /// アスペクト比から先に決めておく (TextureVisualizer も Start で同じ計算を
    /// するが、レイアウト前に走ると幅 0 になるため)。
    /// </summary>
    Component AttachImagePreview(SensorRow row)
    {
        var textureSource = row.info.sensor as UnitySensors.Interface.Sensor.ITextureInterface;
        if (textureSource == null)
        {
            return null;
        }
        bool useTexture1 = row.info.kind == SimulationControl.GuiSensorVizKind.ImageTexture1;
        Texture texture = useTexture1 ? textureSource.texture1 : textureSource.texture0;
        if (texture == null)
        {
            return null;
        }

        float width = m_JointContent.rect.width - 12f; // レイアウトの左右 padding ぶん
        if (width <= 0f)
        {
            width = 264f;
        }

        GameObject previewGo = CreateUiObject(row.info.label + " preview", m_JointContent);
        RectTransform previewRt = previewGo.GetComponent<RectTransform>();
        previewRt.sizeDelta = new Vector2(width, width * texture.height / texture.width);
        previewRt.SetSiblingIndex(row.root.transform.GetSiblingIndex() + 1);
        RawImage image = previewGo.AddComponent<RawImage>();
        image.texture = texture;

        row.previewRow = previewGo;
        var visualizer = row.info.sensor.gameObject
            .AddComponent<UnitySensors.Visualization.Sensor.TextureVisualizer>();
        visualizer.Configure(row.info.sensor, image, useTexture1);
        return visualizer;
    }

    JointRow CreateJointRow(SimulationControl.GuiJointInfo joint)
    {
        GameObject rowGo = CreateUiObject(joint.jointName, m_JointContent);
        rowGo.GetComponent<RectTransform>().sizeDelta = new Vector2(0f, 54f);

        GameObject labelGo = CreateUiObject("Label", rowGo.transform);
        RectTransform labelRt = labelGo.GetComponent<RectTransform>();
        labelRt.anchorMin = new Vector2(0f, 0.5f);
        labelRt.anchorMax = new Vector2(1f, 1f);
        labelRt.offsetMin = new Vector2(4f, 0f);
        labelRt.offsetMax = new Vector2(-4f, 0f);
        TextMeshProUGUI label = labelGo.AddComponent<TextMeshProUGUI>();
        label.fontSize = 14f;
        label.color = TextColor;
        label.alignment = TextAlignmentOptions.MidlineLeft;
        label.textWrappingMode = TextWrappingModes.NoWrap;
        label.overflowMode = TextOverflowModes.Ellipsis;

        Slider slider = CreateSlider(rowGo.GetComponent<RectTransform>());
        RectTransform sliderRt = slider.GetComponent<RectTransform>();
        sliderRt.anchorMin = new Vector2(0f, 0f);
        sliderRt.anchorMax = new Vector2(1f, 0.5f);
        sliderRt.offsetMin = new Vector2(4f, 4f);
        sliderRt.offsetMax = new Vector2(-4f, -2f);

        var row = new JointRow
        {
            joint = joint,
            root = rowGo,
            slider = slider,
            label = label,
            drag = slider.gameObject.AddComponent<SliderDragWatcher>(),
        };

        // スライダの表示単位: 回転関節は度、直動関節はメートル。
        bool prismatic = joint.kind == SimulationControl.GuiJointKind.Prismatic;
        float toDisplay = prismatic ? 1f : Mathf.Rad2Deg;
        SimulationControl.TryGetJointPosition(joint, out float currentSi);
        if (joint.hasLimits)
        {
            row.minDisplay = joint.lowerLimit * toDisplay;
            row.maxDisplay = joint.upperLimit * toDisplay;
        }
        else if (prismatic)
        {
            // リミット無しの直動関節は現在位置 ±0.5 m を可動範囲として見せる。
            row.minDisplay = currentSi - 0.5f;
            row.maxDisplay = currentSi + 0.5f;
        }
        else
        {
            // continuous (とリミット未設定の回転) は ±180 度で表示する。
            row.minDisplay = -180f;
            row.maxDisplay = 180f;
        }

        slider.minValue = row.minDisplay;
        slider.maxValue = row.maxDisplay;
        slider.wholeNumbers = false;
        slider.SetValueWithoutNotify(
            Mathf.Clamp(currentSi * toDisplay, row.minDisplay, row.maxDisplay));
        UpdateRowLabel(row, currentSi * toDisplay);

        slider.onValueChanged.AddListener(value =>
        {
            // ユーザー操作 (またはコード側の SetValue) で呼ばれる。表示追従は
            // SetValueWithoutNotify を使うので、ここに来るのは指令のときだけ。
            float si = prismatic ? value : value * Mathf.Deg2Rad;
            CommandJoint(row.joint, si);
            UpdateRowLabel(row, value);
        });

        return row;
    }

    void UpdateRowLabel(JointRow row, float displayValue)
    {
        // 値が動いたときだけ文字列を作り直す (毎フレームの文字列アロケーション回避)。
        if (!float.IsNaN(row.lastLabelValue)
            && Mathf.Abs(displayValue - row.lastLabelValue) < 0.005f)
        {
            return;
        }
        row.lastLabelValue = displayValue;
        row.label.text = row.joint.kind == SimulationControl.GuiJointKind.Prismatic
            ? $"{row.joint.jointName}  {displayValue:F3} m"
            : $"{row.joint.jointName}  {displayValue:F1} deg";
    }

    /// <summary>
    /// JointStateSub.Callback と同じ書き込み先へ位置指令を出す (速度指令は 0)。
    /// ServoJointModel がある関節は SI のままモデルへ、無い関節は xDrive.target へ
    /// (回転関節は rad→deg、直動関節は m のまま。ArticulationBody の期待単位)。
    /// </summary>
    static void CommandJoint(SimulationControl.GuiJointInfo joint, float positionSi)
    {
        if (joint.servo != null)
        {
            joint.servo.SetCommand(positionSi, 0f);
            return;
        }
        if (joint.body == null)
        {
            return;
        }
        ArticulationDrive drive = joint.body.xDrive;
        drive.target = joint.kind == SimulationControl.GuiJointKind.Prismatic
            ? positionSi
            : positionSi * Mathf.Rad2Deg;
        drive.targetVelocity = 0f;
        joint.body.xDrive = drive;
    }

    void SyncJointRows()
    {
        foreach (JointRow row in m_JointRows)
        {
            if (row.slider == null)
            {
                continue; // "(no movable joints)" のプレースホルダ行
            }
            // ドラッグ中はユーザーの値が勝つ。離れている間だけ関節の現在値に
            // 追従させ、ROS 側からの指令が見えるようにする。
            if (row.drag != null && row.drag.IsDragging)
            {
                continue;
            }
            if (!SimulationControl.TryGetJointPosition(row.joint, out float positionSi))
            {
                continue; // 破棄途中。次の Update の選択チェックで行ごと消える
            }
            bool prismatic = row.joint.kind == SimulationControl.GuiJointKind.Prismatic;
            float display = prismatic ? positionSi : positionSi * Mathf.Rad2Deg;
            row.slider.SetValueWithoutNotify(
                Mathf.Clamp(display, row.minDisplay, row.maxDisplay));
            UpdateRowLabel(row, display);
        }
    }
}

/// <summary>
/// スライダがつかまれているかを見るだけの部品。ハンドルや溝のどこを押しても
/// イベントは Slider ルートまで伝播してくるので、ここに付ければ足りる。
/// </summary>
public class SliderDragWatcher : MonoBehaviour, IPointerDownHandler, IPointerUpHandler
{
    public bool IsDragging { get; private set; }

    public void OnPointerDown(PointerEventData eventData)
    {
        IsDragging = true;
    }

    public void OnPointerUp(PointerEventData eventData)
    {
        IsDragging = false;
    }
}
