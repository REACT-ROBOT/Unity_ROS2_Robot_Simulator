using UnityEngine;
using UnityEngine.UI;      // ← これを追加
using System.Collections.Generic;

public class ToggleLight : MonoBehaviour
{
    [Header("切り替え対象のオブジェクト")]
    [SerializeField] private GameObject targetObject;

    [Header("UI の Toggle")]
    [SerializeField] private Toggle inputToggle;   // ← ここで Toggle を受け取る

    [Header("任意: キー入力でのトグル切り替え")]
    [SerializeField] private KeyCode toggleKey = KeyCode.None;

    private void Start()
    {
        if (inputToggle != null)
        {
            targetObject.transform.rotation = Quaternion.Euler(50, 30, 0);

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
        {
            if (active)
            {
                targetObject.transform.rotation = Quaternion.Euler(50, 30, 0);
            }
            else
            {
                targetObject.transform.rotation = Quaternion.Euler(-90, 0, 0);
            }
        }
    }
}