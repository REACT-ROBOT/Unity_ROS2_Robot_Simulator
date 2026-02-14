using UnityEngine;
using TMPro;               // ← これを追加

public class SolverIterationController : MonoBehaviour
{
    [Header("TMP の Integer Input Field をアサイン")]
    public TMP_InputField iterationInput;  // ← 型を TMP_InputField に

    void Start()
    {
        // 起動時に現在値をセット
        iterationInput.text = Physics.defaultSolverIterations.ToString();
        // フォーカスを外したとき（Enter でも OK）に呼ばれる
        iterationInput.onEndEdit.AddListener(OnIterationInputEnd);
    }

    void OnIterationInputEnd(string value)
    {
        if (int.TryParse(value, out int iter))
        {
            iter = Mathf.Max(1, iter);
            Physics.defaultSolverIterations = iter;
            int velocity_iter = Mathf.Max(1, iter / 4);
            velocity_iter = Mathf.Min(velocity_iter, 8);
            Physics.defaultSolverVelocityIterations = velocity_iter;
            Debug.Log($"SolverIterations = {iter}");
        }
        else
        {
            // 不正入力時は元に戻す
            iterationInput.text = Physics.defaultSolverIterations.ToString();
        }
    }
}
