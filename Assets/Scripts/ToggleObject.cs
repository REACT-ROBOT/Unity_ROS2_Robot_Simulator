using UnityEngine;
using UnityEngine.UI;      // ← これを追加
using System.Collections.Generic;

public class ToggleObject : MonoBehaviour
{
    [Header("切り替え対象のオブジェクト")]
    [SerializeField] private GameObject targetObject;
    [SerializeField] private List<GameObject> additionalTargets = new List<GameObject>();

    [Header("UI の Toggle")]
    [SerializeField] private Toggle inputToggle;   // ← ここで Toggle を受け取る

    [Header("任意: キー入力でのトグル切り替え")]
    [SerializeField] private KeyCode toggleKey = KeyCode.None;

    private void Start()
    {
        if (inputToggle != null)
        {
            // 起動時に現在のチェック状態で有効化／無効化
            SetActive(inputToggle.isOn);
            // チェックが変わったときに呼ぶリスナーを登録
            inputToggle.onValueChanged.AddListener(OnToggleValueChanged);
        }
    }

    private void Update()
    {
        if (toggleKey != KeyCode.None && Input.GetKeyDown(toggleKey))
        {
            // キー入力で手動トグル
            bool newState = !(targetObject?.activeSelf ?? false);
            SetActive(newState);
            // UI 側のチェックも一緒に切り替えたいなら:
            if (inputToggle != null) inputToggle.isOn = newState;
        }
    }

    // Toggle の状態が変わったときに呼ばれる
    private void OnToggleValueChanged(bool isOn)
    {
        SetActive(isOn);
    }

    // 複数オブジェクトの有効／無効を切り替える共通関数
    public void SetActive(bool active)
    {
        if (targetObject != null)
            targetObject.SetActive(active);

        foreach (var obj in additionalTargets)
            if (obj != null)
                obj.SetActive(active);
    }
}