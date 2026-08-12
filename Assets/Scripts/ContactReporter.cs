using UnityEngine;

/// <summary>
/// リンク本体 (ArticulationBody を持つ GameObject) に付けて、衝突イベントを
/// SimulationControl の集計へ転送する。スポーン時に全リンクへ取り付けられる。
/// 複合コライダの衝突コールバックはボディ側 GameObject に届くので、リンク単位で
/// 漏れなく拾える。
/// </summary>
public class ContactReporter : MonoBehaviour
{
    [HideInInspector] public string entityName;
    [HideInInspector] public string linkName;
    [HideInInspector] public SimulationControl control;

    void OnCollisionEnter(Collision collision)
    {
        if (control != null)
        {
            control.ReportContact(entityName, linkName, collision, isEnter: true);
        }
    }

    void OnCollisionStay(Collision collision)
    {
        if (control != null)
        {
            control.ReportContact(entityName, linkName, collision, isEnter: false);
        }
    }
}
