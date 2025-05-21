using UnityEngine;
using UnityEngine.UI;
using System.Collections;

public class SidebarController : MonoBehaviour
{
    [Header("開閉するパネルの RectTransform")]
    public RectTransform sidebar;

    [Header("開閉アニメーション時間")]
    public float duration = 0.3f;

    private Vector2 hiddenPos;
    private Vector2 shownPos;
    private bool isOpen = false;
    private Coroutine animRoutine;

    void Start()
    {
        // hiddenPos はアンカー右端外 (x = width)
        hiddenPos = new Vector2(sidebar.rect.width, 0);
        // shownPos は右端にピタリ (x = 0)
        shownPos = new Vector2(-sidebar.rect.width/2, 0);

        // 初期状態を隠す
        sidebar.anchoredPosition = hiddenPos;
    }

    // ボタンから OnClick イベントで紐付け
    public void ToggleSidebar()
    {
        if (animRoutine != null) StopCoroutine(animRoutine);
        animRoutine = StartCoroutine(AnimateSidebar(isOpen ? hiddenPos : shownPos));
        isOpen = !isOpen;
    }

    private IEnumerator AnimateSidebar(Vector2 target)
    {
        Vector2 start = sidebar.anchoredPosition;
        float elapsed = 0f;
        while (elapsed < duration)
        {
            elapsed += Time.unscaledDeltaTime;
            sidebar.anchoredPosition = Vector2.Lerp(start, target, elapsed / duration);
            yield return null;
        }
        sidebar.anchoredPosition = target;
    }
}

