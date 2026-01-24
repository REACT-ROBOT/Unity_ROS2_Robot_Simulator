using UnityEngine;
using UnityEngine.UI;
using TMPro;
using NaughtyWaterBuoyancy;

public class WaterController : MonoBehaviour
{
    [Header("Water Volume Object")]
    [SerializeField] private GameObject waterVolumeObject;

    [Header("UI Elements")]
    [SerializeField] private Toggle enableToggle;
    [SerializeField] private TMP_InputField heightInputField;

    [Header("Settings")]
    [SerializeField] private float defaultHeight = 0f;
    [SerializeField] private float minHeight = -100f;
    [SerializeField] private float maxHeight = 100f;

    private void Start()
    {
        // Initialize toggle
        if (enableToggle != null)
        {
            //enableToggle.isOn = waterVolumeObject != null && waterVolumeObject.activeSelf;
            enableToggle.isOn = false; // Start with water disabled
            enableToggle.onValueChanged.AddListener(OnEnableToggleChanged);
        }

        // Initialize height input
        if (heightInputField != null)
        {
            if (waterVolumeObject != null)
            {
                heightInputField.text = waterVolumeObject.transform.position.y.ToString("F2");
            }
            else
            {
                heightInputField.text = defaultHeight.ToString("F2");
            }
            heightInputField.onEndEdit.AddListener(OnHeightInputChanged);
        }
    }

    private void OnEnableToggleChanged(bool isOn)
    {
        if (waterVolumeObject != null)
        {
            waterVolumeObject.SetActive(isOn);
        }
    }

    private void OnHeightInputChanged(string value)
    {
        if (waterVolumeObject != null && float.TryParse(value, out float height))
        {
            height = Mathf.Clamp(height, minHeight, maxHeight);
            Vector3 pos = waterVolumeObject.transform.position;
            pos.y = height;
            waterVolumeObject.transform.position = pos;

            // Update input field with clamped value
            heightInputField.text = height.ToString("F2");
        }
    }

    public void SetWaterHeight(float height)
    {
        if (waterVolumeObject != null)
        {
            height = Mathf.Clamp(height, minHeight, maxHeight);
            Vector3 pos = waterVolumeObject.transform.position;
            pos.y = height;
            waterVolumeObject.transform.position = pos;

            if (heightInputField != null)
            {
                heightInputField.text = height.ToString("F2");
            }
        }
    }

    public void SetWaterEnabled(bool enabled)
    {
        if (waterVolumeObject != null)
        {
            waterVolumeObject.SetActive(enabled);
        }

        if (enableToggle != null)
        {
            enableToggle.isOn = enabled;
        }
    }

    public float GetWaterHeight()
    {
        if (waterVolumeObject != null)
        {
            return waterVolumeObject.transform.position.y;
        }
        return defaultHeight;
    }

    public bool IsWaterEnabled()
    {
        if (waterVolumeObject != null)
        {
            return waterVolumeObject.activeSelf;
        }
        return false;
    }
}
