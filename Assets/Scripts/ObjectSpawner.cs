using UnityEngine;
using TMPro;
using UnityEngine.UI;    // Button
using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using Unity.Robotics.UrdfImporter;
using UnityMeshImporter;
using SFB; // StandaloneFileBrowser

[Serializable]
public class SavedObjectData
{
    public string type;
    public float[] position;      // {x,y,z}
    public float[] rotationEuler; // {x,y,z}
    public float[] scale;         // {x,y,z}
    public string meshPath;       // メッシュファイルのパス
    public bool isActive;
}

[Serializable]
public class SavedSceneData
{
    public List<SavedObjectData> objects = new List<SavedObjectData>();
}

public class ObjectSpawner : MonoBehaviour
{
    [Header("UI：生成リスト")]
    public RectTransform listContent;
    public GameObject listItemPrefab;  // Button を含む

    [Header("UI：位置入力")]
    public TMP_InputField inputX;
    public TMP_InputField inputY;
    public TMP_InputField inputZ;
    public TMP_InputField inputRotX;
    public TMP_InputField inputRotY;
    public TMP_InputField inputRotZ;
    public TMP_InputField inputScaleX;
    public TMP_InputField inputScaleY;
    public TMP_InputField inputScaleZ;
    public Button deleteButton;      // 任意

    private class SpawnedObject
    {
        public GameObject gameObject;
        public Button button;
        public string meshPath;
    }

    private Vector3 spawnPosition;
    private Vector3 spawnRotation;
    private Vector3 spawnScale = new Vector3(1f, 1f, 1f);
    private List<SpawnedObject> spawnedObjects = new List<SpawnedObject>();
    private SpawnedObject selectedObject;

    void Start()
    {
        spawnPosition = Vector3.zero;
        inputX.onEndEdit.AddListener(OnInputXInputEnd);
        inputY.onEndEdit.AddListener(OnInputYInputEnd);
        inputZ.onEndEdit.AddListener(OnInputZInputEnd);
        inputRotX.onEndEdit.AddListener(OnInputRotXInputEnd);
        inputRotY.onEndEdit.AddListener(OnInputRotYInputEnd);
        inputRotZ.onEndEdit.AddListener(OnInputRotZInputEnd);
        inputScaleX.onEndEdit.AddListener(OnInputScaleXInputEnd);
        inputScaleY.onEndEdit.AddListener(OnInputScaleYInputEnd);
        inputScaleZ.onEndEdit.AddListener(OnInputScaleZInputEnd);

        deleteButton.onClick.AddListener(() =>
        {
            if (selectedObject != null)
            {
                spawnedObjects.Remove(selectedObject);
                Destroy(selectedObject.gameObject);
                Destroy(selectedObject.button.gameObject);
                selectedObject = null;

                inputX.text = spawnPosition.z.ToString("F2");
                inputY.text = (-spawnPosition.x).ToString("F2");
                inputZ.text = spawnPosition.y.ToString("F2");
                inputRotX.text = (-spawnRotation.z).ToString("F2");
                inputRotY.text = spawnRotation.x.ToString("F2");
                inputRotZ.text = (-spawnRotation.y).ToString("F2");
                inputScaleX.text = spawnScale.z.ToString("F2");
                inputScaleY.text = spawnScale.x.ToString("F2");
                inputScaleZ.text = spawnScale.y.ToString("F2");
            }
        });
    }

    public void SaveSpawnedObjects()
    {
        SavedSceneData savedSceneData = new SavedSceneData();
        foreach (var obj in spawnedObjects)
        {
            SavedObjectData savedObjectData = new SavedObjectData
            {
                type = obj.gameObject.name.Split('_')[0], // 名前からタイプを取得
                position = new float[] { obj.gameObject.transform.position.x, obj.gameObject.transform.position.y, obj.gameObject.transform.position.z },
                rotationEuler = new float[] { obj.gameObject.transform.rotation.eulerAngles.x, obj.gameObject.transform.rotation.eulerAngles.y, obj.gameObject.transform.rotation.eulerAngles.z },
                scale = new float[] { obj.gameObject.transform.localScale.x, obj.gameObject.transform.localScale.y, obj.gameObject.transform.localScale.z },
                meshPath = obj.meshPath,
                isActive = obj.gameObject.activeSelf
            };
            savedSceneData.objects.Add(savedObjectData);
        }
        string json = JsonUtility.ToJson(savedSceneData, prettyPrint: true);
        string path = StandaloneFileBrowser.SaveFilePanel("Save Scene", "", "scene.json", "json");
        if (string.IsNullOrEmpty(path))
        {
            Debug.LogWarning("No file selected.");
            return;
        }
        File.WriteAllText(path, json);
        Debug.Log($"Scene saved to {path}");
    }

    public void LoadSpawnedObjects()
    {
        string path = StandaloneFileBrowser.OpenFilePanel("Load Scene", "", "json", false)[0];
        if (string.IsNullOrEmpty(path))
        {
            Debug.LogWarning("No file selected.");
            return;
        }
        string json = File.ReadAllText(path);
        SavedSceneData savedSceneData = JsonUtility.FromJson<SavedSceneData>(json);
        foreach (var objData in savedSceneData.objects)
        {
            if (objData.type == "Mesh")
            {
                var ob = MeshImporter.Load(objData.meshPath);
                if (ob == null)
                {
                    Debug.LogError("Failed to load object from File.");
                    return;
                }
                ob.name = $"Mesh_{spawnedObjects.Count}";
                ob.transform.position = new Vector3(objData.position[0], objData.position[1], objData.position[2]);
                ob.transform.rotation = Quaternion.Euler(objData.rotationEuler[0], objData.rotationEuler[1], objData.rotationEuler[2]);
                ob.transform.localScale = new Vector3(objData.scale[0], objData.scale[1], objData.scale[2]);
                // 大規模メッシュの場合はLODを設定
                SetupLODForLargeMesh(ob);
                // MeshColliderの設定（当たり判定を有効化）
                SetupMeshColliders(ob);
                // 両面レンダリング用の白いマテリアルを作成して適用
                ApplyDoubleSidedMaterial(ob);
                ob.SetActive(objData.isActive);
                AddListItem(ob, objData.meshPath);
                continue;
            }
            if (objData.type == "Directional Light")
            {
                var ob = new GameObject($"DirectionalLight_{spawnedObjects.Count}");
                var light = ob.AddComponent<Light>();
                light.type = LightType.Directional;
                light.shadows = LightShadows.Soft;
                ob.transform.position = new Vector3(objData.position[0], objData.position[1], objData.position[2]);
                ob.transform.rotation = Quaternion.Euler(objData.rotationEuler[0], objData.rotationEuler[1], objData.rotationEuler[2]);
                ob.SetActive(objData.isActive);
                AddListItem(ob);
                continue;
            }
            if (objData.type == "Point Light")
            {
                var ob = new GameObject($"PointLight_{spawnedObjects.Count}");
                var light = ob.AddComponent<Light>();
                light.type = LightType.Point;
                light.shadows = LightShadows.Soft;
                light.range = 10f;
                light.intensity = 20f;
                ob.transform.position = new Vector3(objData.position[0], objData.position[1], objData.position[2]);
                ob.SetActive(objData.isActive);
                AddListItem(ob);
                continue;
            }
            if (objData.type == "Spot Light")
            {
                var ob = new GameObject($"SpotLight_{spawnedObjects.Count}");
                var light = ob.AddComponent<Light>();
                light.type = LightType.Spot;
                light.shadows = LightShadows.Soft;
                light.range = 15f;
                light.intensity = 1f;
                light.spotAngle = 30f;
                ob.transform.position = new Vector3(objData.position[0], objData.position[1], objData.position[2]);
                ob.transform.rotation = Quaternion.Euler(objData.rotationEuler[0], objData.rotationEuler[1], objData.rotationEuler[2]);
                ob.SetActive(objData.isActive);
                AddListItem(ob);
                continue;
            }
            GameObject go = GameObject.CreatePrimitive((PrimitiveType)System.Enum.Parse(typeof(PrimitiveType), objData.type));
            go.transform.position = new Vector3(objData.position[0], objData.position[1], objData.position[2]);
            go.transform.rotation = Quaternion.Euler(objData.rotationEuler[0], objData.rotationEuler[1], objData.rotationEuler[2]);
            go.transform.localScale = new Vector3(objData.scale[0], objData.scale[1], objData.scale[2]);
            go.SetActive(objData.isActive);
            RegisterSpawn(go);
        }
    }

    // === 公開メソッド：ボタンから呼び出し ===
    public void SpawnCube() => SpawnPrimitive(PrimitiveType.Cube);
    public void SpawnSphere() => SpawnPrimitive(PrimitiveType.Sphere);
    public void SpawnCylinder() => SpawnPrimitive(PrimitiveType.Cylinder);

    // === 内部ロジック ===
    private void SpawnPrimitive(PrimitiveType type)
    {
        var go = GameObject.CreatePrimitive(type);

        if (selectedObject != null)
        {
            // 以前の選択表示をリセット
            foreach (var ob in spawnedObjects)
            {
                var colors = ob.button.colors;
                colors.normalColor = Color.white;
                ob.button.colors = colors;
            }

            selectedObject = null;  // 選択解除

            inputX.text = spawnPosition.z.ToString("F2");
            inputY.text = (-spawnPosition.x).ToString("F2");
            inputZ.text = spawnPosition.y.ToString("F2");
            inputRotX.text = (-spawnRotation.z).ToString("F2");
            inputRotY.text = spawnRotation.x.ToString("F2");
            inputRotZ.text = (-spawnRotation.y).ToString("F2");
            inputScaleX.text = spawnScale.z.ToString("F2");
            inputScaleY.text = spawnScale.x.ToString("F2");
            inputScaleZ.text = spawnScale.y.ToString("F2");
        }
        go.transform.position = spawnPosition;
        go.transform.rotation = Quaternion.Euler(spawnRotation);
        go.transform.localScale = spawnScale;
        RegisterSpawn(go);
    }

    private void RegisterSpawn(GameObject go)
    {
        string baseName = go.name;
        go.name = $"{baseName}_{spawnedObjects.Count}";

        // --- マテリアル設定 ---
        var rend = go.GetComponent<Renderer>();
        if (rend != null)
        {
            // URP Lit シェーダーを使う例
            var shader = Shader.Find("Universal Render Pipeline/Lit");
            if (shader != null)
            {
                rend.material = new Material(shader);
            }
            else
            {
                Debug.LogWarning("Shader URP Lit が見つかりません。");
            }
        }

        AddListItem(go);
    }

    public void SpawnMeshFile()
    {
        // ファイルダイアログを開いて、ファイルを選択する
        // サポートファイルリスト
        // - 3D Manufacturing Format (.3mf)
        // - Collada (.dae, .xml)
        // - Blender (.blend)
        // - Biovision BVH (.bvh)
        // - 3D Studio Max 3DS (.3ds)
        // - 3D Studio Max ASE (.ase)
        // - glTF (.glTF)
        // - glTF2.0 (.glTF)
        // - KHR_lights_punctual ( 5.0 )
        // - KHR_materials_pbrSpecularGlossiness ( 5.0 )
        // - KHR_materials_unlit ( 5.0 )
        // - KHR_texture_transform ( 5.1 under test )
        // - FBX-Format, as ASCII and binary (.fbx)
        // - Stanford Polygon Library (.ply)
        // - AutoCAD DXF (.dxf)
        // - IFC-STEP (.ifc)
        // - Neutral File Format (.nff)
        // - Sense8 WorldToolkit (.nff)
        // - Valve Model (.smd, .vta)
        // - Quake I (.mdl)
        // - Quake II (.md2)
        // - Quake III (.md3)
        // - Quake 3 BSP (.pk3)
        // - RtCW (.mdc)
        // - Doom 3 (.md5mesh, .md5anim, .md5camera)
        // - DirectX X (.x)
        // - Quick3D (.q3o, .q3s)
        // - Raw Triangles (.raw)
        // - AC3D (.ac, .ac3d)
        // - Stereolithography (.stl)
        // - Autodesk DXF (.dxf)
        // - Irrlicht Mesh (.irrmesh, .xml)
        // - Irrlicht Scene (.irr, .xml)
        // - Object File Format ( .off )
        // - Wavefront Object (.obj)
        // - Terragen Terrain ( .ter )
        // - 3D GameStudio Model ( .mdl )
        // - 3D GameStudio Terrain ( .hmp )
        // - Ogre ( .mesh.xml, .skeleton.xml, .material )
        // - OpenGEX-Fomat (.ogex)
        // - Milkshape 3D ( .ms3d )
        // - LightWave Model ( .lwo )
        // - LightWave Scene ( .lws )
        // - Modo Model ( .lxo )
        // - CharacterStudio Motion ( .csm )
        // - Stanford Ply ( .ply )
        // - TrueSpace (.cob, .scn)
        // - XGL-3D-Format (.xgl)
        // ExtensionFilter配列を使用（Linux版のエンコーディング問題を回避）
        var extensions = new SFB.ExtensionFilter[] {
            new SFB.ExtensionFilter("3D Models", new string[] {
                "3mf", "dae", "blend", "bvh", "3ds", "ase", "gltf", "glb",
                "fbx", "ply", "dxf", "ifc", "nff", "smd", "vta", "mdl",
                "md2", "md3", "pk3", "mdc", "md5mesh", "x",
                "q3o", "q3s", "raw", "ac", "stl"
            }),
            new SFB.ExtensionFilter("All Files", "*")
        };
        // 複数ファイル選択を有効化（最後の引数をtrueに）
        string[] filePaths = null;
        try
        {
            filePaths = StandaloneFileBrowser.OpenFilePanel("Select Mesh File(s)", "", extensions, true);
            Debug.Log($"[DEBUG] OpenFilePanel returned: filePaths={filePaths}, Length={filePaths?.Length ?? -1}");
            if (filePaths != null && filePaths.Length > 0)
            {
                Debug.Log($"[DEBUG] First path: \"{filePaths[0]}\"");
            }
        }
        catch (System.Exception e)
        {
            // Linux/WSL2環境で大きなファイルや多数のファイルを選択するとクラッシュすることがある
            Debug.LogError($"File selection failed: {e.Message}");
            Debug.LogWarning("ヒント: 大きなファイルや多数のファイルを読み込む場合は、[Mesh Folder]ボタンでフォルダを選択してください。");
            return;
        }

        // ファイル選択がキャンセルまたは失敗した場合
        // 空文字列の配列が返される場合もあるため、最初の要素もチェック
        if (filePaths == null || filePaths.Length == 0 ||
            (filePaths.Length == 1 && string.IsNullOrEmpty(filePaths[0])))
        {
            Debug.Log("No file selected.");
            // 注意: Linux/WSL2環境ではGTKダイアログを連続で開くとクラッシュするため
            // フォルダ選択は別のボタン(SpawnMeshFolder)を使用してください
            return;
        }

        Debug.Log($"Selected {filePaths.Length} file(s):");

        // ファイルパスをクリーンアップしてコピー
        // Linux/WSL2環境でStandaloneFileBrowserがファイル数が多い場合に
        // パスの先頭に制御文字や謎の文字を追加するバグを回避
        List<string> pathList = new List<string>();
        for (int i = 0; i < filePaths.Length; i++)
        {
            string rawPath = filePaths[i];
            string cleanedPath = CleanFilePath(rawPath);
            Debug.Log($"  [{i}]: raw=\"{rawPath}\" -> cleaned=\"{cleanedPath}\"");
            if (!string.IsNullOrEmpty(cleanedPath))
            {
                pathList.Add(cleanedPath);
            }
        }

        if (pathList.Count == 0)
        {
            Debug.LogError("No valid file paths after cleanup.");
            return;
        }

        Debug.Log($"Valid paths after cleanup: {pathList.Count}");

        // コルーチンで順次ロード（WSL2のファイルシステム問題を回避）
        StartCoroutine(LoadMeshFilesCoroutine(pathList));
    }

    // サポートされているメッシュファイルの拡張子
    private static readonly HashSet<string> SupportedMeshExtensions = new HashSet<string>(StringComparer.OrdinalIgnoreCase)
    {
        ".3mf", ".dae", ".blend", ".bvh", ".3ds", ".ase", ".gltf", ".glb",
        ".fbx", ".ply", ".dxf", ".ifc", ".nff", ".smd", ".vta", ".mdl",
        ".md2", ".md3", ".pk3", ".mdc", ".md5mesh", ".x",
        ".q3o", ".q3s", ".raw", ".ac", ".stl", ".obj"
    };

    /// <summary>
    /// フォルダを選択して、その中のすべてのメッシュファイルを読み込む
    /// Linux/WSL2環境でStandaloneFileBrowserの複数ファイル選択がエンコーディングエラーを起こす問題を回避
    /// </summary>
    public void SpawnMeshFolder()
    {
        string[] folderPaths = StandaloneFileBrowser.OpenFolderPanel("Select Mesh Folder", "", false);
        if (folderPaths == null || folderPaths.Length == 0 || string.IsNullOrEmpty(folderPaths[0]))
        {
            Debug.LogWarning("No folder selected.");
            return;
        }
        string folderPath = folderPaths[0];

        Debug.Log($"Selected folder: {folderPath}");

        // フォルダ内のメッシュファイルを検索
        List<string> meshFiles = new List<string>();
        try
        {
            string[] allFiles = Directory.GetFiles(folderPath);
            foreach (string file in allFiles)
            {
                string extension = Path.GetExtension(file);
                if (SupportedMeshExtensions.Contains(extension))
                {
                    meshFiles.Add(file);
                }
            }
        }
        catch (System.Exception e)
        {
            Debug.LogError($"Failed to read folder: {e.Message}");
            return;
        }

        if (meshFiles.Count == 0)
        {
            Debug.LogWarning($"No mesh files found in folder: {folderPath}");
            return;
        }

        // ファイル名でソート（一貫した順序でロード）
        meshFiles.Sort();

        Debug.Log($"Found {meshFiles.Count} mesh files in folder");
        for (int i = 0; i < meshFiles.Count; i++)
        {
            Debug.Log($"  [{i}]: {Path.GetFileName(meshFiles[i])}");
        }

        // コルーチンで順次ロード
        StartCoroutine(LoadMeshFilesCoroutine(meshFiles));
    }

    /// <summary>
    /// 複数のメッシュファイルを順次ロードするコルーチン
    /// WSL2環境でのファイルアクセス問題を回避するため、各ファイル間で待機
    /// </summary>
    private IEnumerator LoadMeshFilesCoroutine(List<string> filePaths)
    {
        int loadedCount = 0;
        int failedCount = 0;

        foreach (string filePath in filePaths)
        {
            if (string.IsNullOrEmpty(filePath))
            {
                continue;
            }

            Debug.Log($"Loading: {filePath}");
            Debug.Log($"[DEBUG] Before WaitForSecondsRealtime (TimeScale={Time.timeScale})");

            // WSL2のファイルシステム同期問題を回避するため、0.1秒待機
            // WaitForSecondsRealtimeを使用してTimeScaleに依存しない待機
            yield return new WaitForSecondsRealtime(0.1f);

            Debug.Log($"[DEBUG] After WaitForSeconds, checking file existence");

            // ファイルの存在確認（リトライ付き）
            bool fileExists = false;
            for (int retry = 0; retry < 5; retry++)
            {
                try
                {
                    fileExists = File.Exists(filePath);
                    Debug.Log($"[DEBUG] File.Exists returned: {fileExists}");
                }
                catch (System.Exception e)
                {
                    Debug.LogError($"[DEBUG] File.Exists threw exception: {e.Message}");
                }

                if (fileExists)
                {
                    break;
                }
                Debug.Log($"File not found, retrying ({retry + 1}/5): {filePath}");
                yield return new WaitForSecondsRealtime(0.1f);
            }

            if (!fileExists)
            {
                Debug.LogError($"File does not exist after retries: {filePath}");
                failedCount++;
                continue;
            }

            // ファイルサイズ確認
            Debug.Log($"[DEBUG] Getting file info");
            FileInfo fileInfo = null;
            try
            {
                fileInfo = new FileInfo(filePath);
                Debug.Log($"File size: {fileInfo.Length} bytes");
            }
            catch (System.Exception e)
            {
                Debug.LogError($"[DEBUG] FileInfo threw exception: {e.Message}");
                failedCount++;
                continue;
            }

            GameObject ob = null;
            try
            {
                ob = MeshImporter.Load(filePath);
            }
            catch (System.Exception e)
            {
                Debug.LogError($"Exception loading file: {filePath}\n{e.Message}\n{e.StackTrace}");
                failedCount++;
                continue;
            }

            if (ob == null)
            {
                Debug.LogError($"Failed to load object from File (returned null): {filePath}");
                failedCount++;
                continue;
            }
            ob.name = $"Mesh_{spawnedObjects.Count}";

            // 静的オブジェクトとしてマーク（Static Batching有効化）
            // これによりdraw callが削減され、パフォーマンスが向上
            SetStaticRecursively(ob);

            // 大規模メッシュの場合はLODを設定してパフォーマンス改善
            SetupLODForLargeMesh(ob);

            // MeshColliderの設定（当たり判定を有効化）
            SetupMeshColliders(ob);

            // 両面レンダリング用の白いマテリアルを作成して適用
            ApplyDoubleSidedMaterial(ob);

            ob.transform.position = spawnPosition;
            ob.transform.rotation = Quaternion.Euler(spawnRotation);
            ob.transform.localScale = spawnScale;

            AddListItem(ob, filePath);
            loadedCount++;

            // 処理負荷を分散するため、1フレーム待機
            yield return null;
        }

        Debug.Log($"Mesh loading complete: {loadedCount} loaded, {failedCount} failed, {filePaths.Count} total.");
    }

    /// <summary>Directional Light を生成</summary>
    public void SpawnDirectionalLight()
    {
        var go = new GameObject($"DirectionalLight_{spawnedObjects.Count}");
        var light = go.AddComponent<Light>();
        light.type = LightType.Directional;
        light.shadows = LightShadows.Soft;
        go.transform.position = spawnPosition;
        go.transform.rotation = Quaternion.Euler(spawnRotation);
        AddListItem(go);
    }

    /// <summary>Point Light を生成</summary>
    public void SpawnPointLight()
    {
        var go = new GameObject($"PointLight_{spawnedObjects.Count}");
        var light = go.AddComponent<Light>();
        light.type = LightType.Point;
        light.shadows = LightShadows.Soft;
        light.range = 10f;
        light.intensity = 20f;
        go.transform.position = spawnPosition;
        AddListItem(go);
    }

    /// <summary>Spot Light を生成</summary>
    public void SpawnSpotLight()
    {
        var go = new GameObject($"SpotLight_{spawnedObjects.Count}");
        var light = go.AddComponent<Light>();
        light.type = LightType.Spot;
        light.shadows = LightShadows.Soft;
        light.range = 15f;
        light.intensity = 1f;
        light.spotAngle = 30f;
        go.transform.position = spawnPosition;
        go.transform.rotation = Quaternion.Euler(spawnRotation);
        AddListItem(go);
    }

    /// <summary>
    /// ファイルパスから不正な文字を除去してクリーンなパスを返す
    /// Linux/WSL2環境でStandaloneFileBrowserが多数のファイルを選択した際に
    /// パスの先頭に制御文字や記号を付加するバグに対応
    /// </summary>
    private string CleanFilePath(string rawPath)
    {
        if (string.IsNullOrEmpty(rawPath))
        {
            return null;
        }

        // パス内の "/home/" または "/" で始まる絶対パスを探す
        // 先頭に不正な文字が付いている場合、それを除去する
        int homeIndex = rawPath.IndexOf("/home/");
        if (homeIndex >= 0)
        {
            return rawPath.Substring(homeIndex);
        }

        // "/home/" が見つからない場合は、最初の "/" を探す
        // ただし、先頭の不正文字をスキップする
        for (int i = 0; i < rawPath.Length; i++)
        {
            char c = rawPath[i];
            // 有効なパス開始文字（/、~、または英数字）を探す
            if (c == '/')
            {
                // 次の文字が "/" でない場合のみ有効（"//" は無効なパス）
                if (i + 1 < rawPath.Length && rawPath[i + 1] != '/')
                {
                    return rawPath.Substring(i);
                }
            }
        }

        // クリーンアップできなかった場合は空文字を返す
        Debug.LogWarning($"Could not clean file path: {rawPath}");
        return null;
    }

    // コリジョン用の簡略化メッシュの最大三角形数
    private const int MAX_COLLISION_TRIANGLES = 50000;
    private const float COLLISION_RATIO = 0.2f;

    // LOD用の閾値（この三角形数を超えるとLODを生成）
    // 非常に小さいメッシュ（100三角形未満）は対象外とする
    // それ以上のメッシュは画面占有率ベースでLODが切り替わる
    private const int LOD_THRESHOLD_TRIANGLES = 100;

    /// <summary>
    /// GameObjectとその子を再帰的に静的オブジェクトとしてマーク
    /// Static Batchingを有効にしてdraw callを削減
    /// </summary>
    private void SetStaticRecursively(GameObject obj)
    {
        obj.isStatic = true;
        foreach (Transform child in obj.transform)
        {
            SetStaticRecursively(child.gameObject);
        }
    }

    /// <summary>
    /// 大規模メッシュにLODシステムを設定してレンダリングパフォーマンスを改善
    /// </summary>
    private void SetupLODForLargeMesh(GameObject obj)
    {
        MeshFilter[] meshFilters = obj.GetComponentsInChildren<MeshFilter>();

        foreach (MeshFilter meshFilter in meshFilters)
        {
            Mesh mesh = meshFilter.sharedMesh;
            if (mesh == null) continue;

            int triangleCount = mesh.triangles.Length / 3;
            if (triangleCount < LOD_THRESHOLD_TRIANGLES) continue;

            Debug.Log($"Setting up LOD for large mesh: {meshFilter.gameObject.name}, triangles: {triangleCount}");

            GameObject meshObj = meshFilter.gameObject;
            MeshRenderer originalRenderer = meshObj.GetComponent<MeshRenderer>();
            if (originalRenderer == null) continue;

            // LODグループを親オブジェクトに追加
            LODGroup lodGroup = meshObj.AddComponent<LODGroup>();

            // LOD0: オリジナルメッシュ（近距離）
            // LOD1: 10%に簡略化（中距離）
            // LOD2: 1%に簡略化（遠距離）
            // Culled: 完全に非表示（非常に遠い）

            // LOD1用の簡略化メッシュを作成
            Mesh lod1Mesh = CreateSimplifiedCollisionMesh(mesh, triangleCount / 10);
            GameObject lod1Obj = new GameObject("LOD1");
            lod1Obj.transform.SetParent(meshObj.transform, false);
            MeshFilter lod1Filter = lod1Obj.AddComponent<MeshFilter>();
            lod1Filter.sharedMesh = lod1Mesh;
            MeshRenderer lod1Renderer = lod1Obj.AddComponent<MeshRenderer>();
            lod1Renderer.sharedMaterials = originalRenderer.sharedMaterials;

            // LOD2用の簡略化メッシュを作成
            Mesh lod2Mesh = CreateSimplifiedCollisionMesh(mesh, triangleCount / 100);
            GameObject lod2Obj = new GameObject("LOD2");
            lod2Obj.transform.SetParent(meshObj.transform, false);
            MeshFilter lod2Filter = lod2Obj.AddComponent<MeshFilter>();
            lod2Filter.sharedMesh = lod2Mesh;
            MeshRenderer lod2Renderer = lod2Obj.AddComponent<MeshRenderer>();
            lod2Renderer.sharedMaterials = originalRenderer.sharedMaterials;

            // LODレベルを設定
            // 値は画面上の相対サイズ（0.0〜1.0）で、この値以上の場合にそのLODが使用される
            // より早くLODが切り替わるように閾値を上げる
            LOD[] lods = new LOD[3];
            lods[0] = new LOD(0.15f, new Renderer[] { originalRenderer });  // 画面の15%以上: フル詳細
            lods[1] = new LOD(0.05f, new Renderer[] { lod1Renderer });      // 画面の5-15%: 10%詳細
            lods[2] = new LOD(0.01f, new Renderer[] { lod2Renderer });      // 画面の1-5%: 1%詳細
            // 1%未満: カリング（非表示）

            lodGroup.SetLODs(lods);
            lodGroup.RecalculateBounds();

            // LODBiasを調整（値が小さいほど低LODに切り替わりやすい）
            // QualitySettings.lodBias はグローバル設定なのでここでは変更しない

            Debug.Log($"LOD setup complete: LOD0={triangleCount}, LOD1={lod1Mesh.triangles.Length / 3}, LOD2={lod2Mesh.triangles.Length / 3} triangles");
        }
    }

    /// <summary>
    /// MeshFilterを持つすべての子オブジェクトにMeshColliderを設定する
    /// 大規模メッシュは自動的に簡略化してパフォーマンスを改善
    /// </summary>
    private void SetupMeshColliders(GameObject obj)
    {
        Debug.Log($"SetupMeshColliders called for: {obj.name}");

        // 既存のMeshColliderを処理（MeshImporterが追加した場合）
        MeshCollider[] existingColliders = obj.GetComponentsInChildren<MeshCollider>();
        Debug.Log($"Found {existingColliders.Length} existing MeshColliders");

        foreach (MeshCollider meshCollider in existingColliders)
        {
            MeshFilter meshFilter = meshCollider.gameObject.GetComponent<MeshFilter>();
            if (meshFilter == null)
            {
                Debug.LogWarning($"MeshCollider on {meshCollider.gameObject.name} has no MeshFilter");
                continue;
            }

            Mesh mesh = meshFilter.sharedMesh;
            if (mesh == null)
            {
                Debug.LogWarning($"MeshFilter on {meshFilter.gameObject.name} has no mesh");
                continue;
            }

            int triangleCount = mesh.triangles.Length / 3;
            Debug.Log($"Processing {meshCollider.gameObject.name}, mesh: {mesh.name}, triangles: {triangleCount}");

            // 大規模メッシュは簡略化してコリジョン用メッシュを生成
            Mesh collisionMesh = mesh;
            if (triangleCount > MAX_COLLISION_TRIANGLES)
            {
                Debug.Log($"Large mesh detected. Creating simplified collision mesh (target: {MAX_COLLISION_TRIANGLES} triangles)");
                collisionMesh = CreateSimplifiedCollisionMesh(mesh, (int)(triangleCount * COLLISION_RATIO));
            }

            meshCollider.sharedMesh = null;
            meshCollider.sharedMesh = collisionMesh;
            Debug.Log($"MeshCollider configured: {meshCollider.gameObject.name}, collision triangles: {collisionMesh.triangles.Length / 3}");
        }

        // MeshColliderが存在しないMeshFilterに対して新規追加
        MeshFilter[] meshFilters = obj.GetComponentsInChildren<MeshFilter>();
        Debug.Log($"Found {meshFilters.Length} MeshFilters total");

        foreach (MeshFilter meshFilter in meshFilters)
        {
            Mesh mesh = meshFilter.sharedMesh;
            if (mesh == null)
            {
                Debug.LogWarning($"Skipping {meshFilter.gameObject.name}: no mesh");
                continue;
            }

            Collider existingCollider = meshFilter.gameObject.GetComponent<Collider>();
            if (existingCollider != null)
            {
                Debug.Log($"Skipping {meshFilter.gameObject.name}: already has collider");
                continue;
            }

            int triangleCount = mesh.triangles.Length / 3;
            Debug.Log($"Adding MeshCollider to: {meshFilter.gameObject.name}, triangles: {triangleCount}");

            MeshCollider meshCollider = meshFilter.gameObject.AddComponent<MeshCollider>();

            // 大規模メッシュは簡略化
            Mesh collisionMesh = mesh;
            if (triangleCount > MAX_COLLISION_TRIANGLES)
            {
                Debug.Log($"Large mesh detected. Creating simplified collision mesh (target: {MAX_COLLISION_TRIANGLES} triangles)");
                collisionMesh = CreateSimplifiedCollisionMesh(mesh, MAX_COLLISION_TRIANGLES);
            }

            meshCollider.sharedMesh = collisionMesh;
            Debug.Log($"MeshCollider added: {meshFilter.gameObject.name}, collision triangles: {collisionMesh.triangles.Length / 3}");
        }
    }

    /// <summary>
    /// メッシュを簡略化してLOD/コリジョン用の軽量メッシュを生成
    /// 頂点クラスタリングアルゴリズムを使用して、穴のない連続した面を維持
    /// </summary>
    private Mesh CreateSimplifiedCollisionMesh(Mesh originalMesh, int targetTriangles)
    {
        Vector3[] vertices = originalMesh.vertices;
        int[] triangles = originalMesh.triangles;
        int originalTriangleCount = triangles.Length / 3;

        if (originalTriangleCount <= targetTriangles)
        {
            return originalMesh;
        }

        // 目標の頂点数を推定（三角形数の約半分）
        int targetVertices = Mathf.Max(targetTriangles / 2, 4);

        // メッシュのバウンディングボックスを計算
        Bounds bounds = originalMesh.bounds;
        Vector3 size = bounds.size;
        Vector3 min = bounds.min;

        // グリッドの解像度を計算（目標頂点数に基づく）
        float volume = size.x * size.y * size.z;
        float cellSize = Mathf.Pow(volume / targetVertices, 1f / 3f);

        // 最小セルサイズを設定（極端に小さくならないように）
        cellSize = Mathf.Max(cellSize, Mathf.Min(size.x, Mathf.Min(size.y, size.z)) / 100f);

        int gridX = Mathf.Max(1, Mathf.CeilToInt(size.x / cellSize));
        int gridY = Mathf.Max(1, Mathf.CeilToInt(size.y / cellSize));
        int gridZ = Mathf.Max(1, Mathf.CeilToInt(size.z / cellSize));

        // 頂点をグリッドセルにマッピング
        // キー: グリッドセルのインデックス、値: (頂点位置の合計, 頂点数)
        Dictionary<int, (Vector3 sum, int count)> cellVertices = new Dictionary<int, (Vector3, int)>();
        Dictionary<int, int> vertexToCellIndex = new Dictionary<int, int>();

        for (int i = 0; i < vertices.Length; i++)
        {
            Vector3 v = vertices[i];

            // 頂点のグリッドセル座標を計算
            int cx = Mathf.Clamp(Mathf.FloorToInt((v.x - min.x) / cellSize), 0, gridX - 1);
            int cy = Mathf.Clamp(Mathf.FloorToInt((v.y - min.y) / cellSize), 0, gridY - 1);
            int cz = Mathf.Clamp(Mathf.FloorToInt((v.z - min.z) / cellSize), 0, gridZ - 1);

            int cellIndex = cx + cy * gridX + cz * gridX * gridY;
            vertexToCellIndex[i] = cellIndex;

            if (cellVertices.TryGetValue(cellIndex, out var cell))
            {
                cellVertices[cellIndex] = (cell.sum + v, cell.count + 1);
            }
            else
            {
                cellVertices[cellIndex] = (v, 1);
            }
        }

        // 各セルの代表頂点（平均位置）を計算
        Dictionary<int, int> cellToNewVertex = new Dictionary<int, int>();
        List<Vector3> newVertices = new List<Vector3>();

        foreach (var kvp in cellVertices)
        {
            Vector3 avgPosition = kvp.Value.sum / kvp.Value.count;
            cellToNewVertex[kvp.Key] = newVertices.Count;
            newVertices.Add(avgPosition);
        }

        // 三角形を新しい頂点インデックスで再構築
        List<int> newTriangles = new List<int>();
        HashSet<(int, int, int)> addedTriangles = new HashSet<(int, int, int)>();

        for (int i = 0; i < originalTriangleCount; i++)
        {
            int idx = i * 3;
            int v0 = triangles[idx];
            int v1 = triangles[idx + 1];
            int v2 = triangles[idx + 2];

            int newV0 = cellToNewVertex[vertexToCellIndex[v0]];
            int newV1 = cellToNewVertex[vertexToCellIndex[v1]];
            int newV2 = cellToNewVertex[vertexToCellIndex[v2]];

            // 縮退三角形をスキップ（3頂点が同じセルに属する場合）
            if (newV0 == newV1 || newV1 == newV2 || newV2 == newV0)
            {
                continue;
            }

            // 重複三角形をスキップ（頂点をソートして比較）
            var sortedTri = SortTriangle(newV0, newV1, newV2);
            if (addedTriangles.Contains(sortedTri))
            {
                continue;
            }
            addedTriangles.Add(sortedTri);

            newTriangles.Add(newV0);
            newTriangles.Add(newV1);
            newTriangles.Add(newV2);
        }

        Mesh simplifiedMesh = new Mesh();
        simplifiedMesh.name = originalMesh.name + "_simplified";

        if (newVertices.Count > 65535)
        {
            simplifiedMesh.indexFormat = UnityEngine.Rendering.IndexFormat.UInt32;
        }

        simplifiedMesh.vertices = newVertices.ToArray();
        simplifiedMesh.triangles = newTriangles.ToArray();
        simplifiedMesh.RecalculateNormals();
        simplifiedMesh.RecalculateBounds();

        int newTriangleCount = newTriangles.Count / 3;
        Debug.Log($"Simplified mesh (vertex clustering): {originalTriangleCount} -> {newTriangleCount} triangles, {vertices.Length} -> {newVertices.Count} vertices");
        return simplifiedMesh;
    }

    /// <summary>
    /// 三角形の頂点インデックスをソートして正規化（重複検出用）
    /// </summary>
    private (int, int, int) SortTriangle(int v0, int v1, int v2)
    {
        if (v0 > v1) (v0, v1) = (v1, v0);
        if (v1 > v2) (v1, v2) = (v2, v1);
        if (v0 > v1) (v0, v1) = (v1, v0);
        return (v0, v1, v2);
    }

    /// <summary>
    /// メッシュに両面レンダリング用の白いマテリアルを適用
    /// </summary>
    private void ApplyDoubleSidedMaterial(GameObject obj)
    {
        // カスタム両面シェーダー（Shader Graph）を優先的に使用
        // Resourcesフォルダからの読み込みも試みる（ビルド時の確実性向上）
        Shader shader = Resources.Load<Shader>("Shaders/DoubleSidedLit");
        if (shader == null)
        {
            shader = Shader.Find("Shader Graphs/DoubleSidedLit");
        }
        if (shader == null)
        {
            shader = Shader.Find("Custom/DoubleSidedLit");
        }
        if (shader == null)
        {
            Debug.LogWarning("Custom DoubleSided shader not found. Using URP Lit shader.");
            shader = Shader.Find("Universal Render Pipeline/Lit");
        }
        if (shader == null)
        {
            Debug.LogWarning("URP Lit shader not found. Trying Standard shader.");
            shader = Shader.Find("Standard");
        }
        
        if (shader != null)
        {
            Material doubleSidedMaterial = new Material(shader);
            doubleSidedMaterial.color = Color.white;
            // 両面レンダリングを有効化
            doubleSidedMaterial.SetInt("_Cull", (int)UnityEngine.Rendering.CullMode.Off);
            
            // 表裏で似た見え方にするための設定
            // Smoothnessを適度に設定（拡散反射を強めにする）
            if (doubleSidedMaterial.HasProperty("_Smoothness"))
            {
                doubleSidedMaterial.SetFloat("_Smoothness", 0.3f);
            }
            // Metallicは0のままで非金属に
            if (doubleSidedMaterial.HasProperty("_Metallic"))
            {
                doubleSidedMaterial.SetFloat("_Metallic", 0.0f);
            }
            // 環境光の影響を強めて表裏の陰影差を軽減
            if (doubleSidedMaterial.HasProperty("_EnvironmentReflections"))
            {
                doubleSidedMaterial.SetFloat("_EnvironmentReflections", 1.0f);
            }
            // 裏面の法線を反転（URPで利用可能な場合）
            if (doubleSidedMaterial.HasProperty("_DoubleSidedNormalMode"))
            {
                // Mirror mode: 裏面で法線を反転
                doubleSidedMaterial.SetFloat("_DoubleSidedNormalMode", 2.0f);
            }
            if (doubleSidedMaterial.HasProperty("_DoubleSidedEnable"))
            {
                doubleSidedMaterial.SetFloat("_DoubleSidedEnable", 1.0f);
            }

            // GPU Instancingを有効化（パフォーマンス改善）
            doubleSidedMaterial.enableInstancing = true;

            // すべてのMeshRendererに適用
            MeshRenderer[] renderers = obj.GetComponentsInChildren<MeshRenderer>();
            foreach (MeshRenderer renderer in renderers)
            {
                Material[] materials = new Material[renderer.sharedMaterials.Length];
                for (int i = 0; i < materials.Length; i++)
                {
                    // 元のマテリアルから新しいマテリアルを作成
                    Material newMaterial = new Material(doubleSidedMaterial);
                    Material originalMaterial = renderer.sharedMaterials[i];
                    
                    if (originalMaterial != null)
                    {
                        // 元のマテリアルの色を保持
                        if (originalMaterial.HasProperty("_Color"))
                        {
                            newMaterial.color = originalMaterial.color;
                        }
                        if (originalMaterial.HasProperty("_BaseColor"))
                        {
                            newMaterial.SetColor("_BaseColor", originalMaterial.GetColor("_BaseColor"));
                        }
                        
                        // 元のマテリアルのメインテクスチャを保持
                        if (originalMaterial.mainTexture != null && newMaterial.HasProperty("_BaseMap"))
                        {
                            newMaterial.SetTexture("_BaseMap", originalMaterial.mainTexture);
                        }
                        else if (originalMaterial.mainTexture != null && newMaterial.HasProperty("_MainTex"))
                        {
                            newMaterial.SetTexture("_MainTex", originalMaterial.mainTexture);
                        }
                    }
                    
                    materials[i] = newMaterial;
                }
                renderer.materials = materials;
            }
        }
        else
        {
            Debug.LogError("No suitable shader found for double-sided material.");
        }
    }

    // リストに行アイテムを追加して、ボタンに選択イベントを登録
    private void AddListItem(GameObject go, string meshPath = null)
    {
        var item = Instantiate(listItemPrefab, listContent);
        var text = item.GetComponentInChildren<TMP_Text>();
        if (text != null) text.text = go.name;

        var btn = item.GetComponent<Button>();
        if (btn != null)
        {
            spawnedObjects.Add(new SpawnedObject { gameObject = go, button = btn, meshPath = meshPath });
            btn.onClick.AddListener(() => OnSelectObject(go, btn));
        }
    }

    // オブジェクト選択時の処理
    private void OnSelectObject(GameObject go, Button btn)
    {
        // 以前の選択表示をリセット
        foreach (var ob in spawnedObjects)
        {
            var colors = ob.button.colors;
            colors.normalColor = Color.white;
            ob.button.colors = colors;
            ob.button.image.color = Color.white;
        }

        if (selectedObject != null)
        {
            if (selectedObject.gameObject == go)
            {
                selectedObject = null;  // 選択解除

                inputX.text = spawnPosition.z.ToString("F2");
                inputY.text = (-spawnPosition.x).ToString("F2");
                inputZ.text = spawnPosition.y.ToString("F2");
                inputRotX.text = (-spawnRotation.z).ToString("F2");
                inputRotY.text = spawnRotation.x.ToString("F2");
                inputRotZ.text = (-spawnRotation.y).ToString("F2");
                inputScaleX.text = spawnScale.z.ToString("F2");
                inputScaleY.text = spawnScale.x.ToString("F2");
                inputScaleZ.text = spawnScale.y.ToString("F2");

                return;
            }
        }

        // このボタンをハイライト
        var cb = btn.colors;
        cb.normalColor = Color.cyan;
        btn.colors = cb;
        btn.image.color = Color.cyan;

        // 選択オブジェクトを設定し、入力欄に現状の位置を表示
        var so = spawnedObjects.Find(x => x.gameObject == go);
        if (so != null) selectedObject = so;
        var pos = go.transform.position;
        var rot = go.transform.rotation.eulerAngles;
        var scale = go.transform.localScale;
        inputX.text = pos.z.ToString("F2");
        inputY.text = (-pos.x).ToString("F2");
        inputZ.text = pos.y.ToString("F2");
        inputRotX.text = (-rot.z).ToString("F2");
        inputRotY.text = rot.x.ToString("F2");
        inputRotZ.text = (-rot.y).ToString("F2");
        inputScaleX.text = scale.z.ToString("F2");
        inputScaleY.text = scale.x.ToString("F2");
        inputScaleZ.text = scale.y.ToString("F2");
    }

    private void OnInputXInputEnd(string value)
    {
        if (selectedObject != null)
        {
            var pos = selectedObject.gameObject.transform.position;
            if (float.TryParse(value, out float x))
            {
                pos.z = x;
            }
            selectedObject.gameObject.transform.position = pos;
        }
        else
        {
            if (float.TryParse(value, out float x))
            {
                spawnPosition.z = x;
            }
            else
            {
                // 不正入力時は元の値に戻す
                inputX.text = spawnPosition.z.ToString("F2");
            }
        }
    }

    private void OnInputYInputEnd(string value)
    {
        if (selectedObject != null)
        {
            var pos = selectedObject.gameObject.transform.position;
            if (float.TryParse(value, out float y))
            {
                pos.x = -y;
            }
            selectedObject.gameObject.transform.position = pos;
        }
        else
        {
            if (float.TryParse(value, out float y))
            {
                spawnPosition.x = -y;
            }
            else
            {
                // 不正入力時は元の値に戻す
                inputY.text = (-spawnPosition.x).ToString("F2");
            }
        }
    }

    private void OnInputZInputEnd(string value)
    {
        if (selectedObject != null)
        {
            var pos = selectedObject.gameObject.transform.position;
            if (float.TryParse(value, out float z))
            {
                pos.y = z;
            }
            selectedObject.gameObject.transform.position = pos;
        }
        else
        {
            if (float.TryParse(value, out float z))
            {
                spawnPosition.y = z;
            }
            else
            {
                // 不正入力時は元の値に戻す
                inputZ.text = spawnPosition.y.ToString("F2");
            }
        }
    }

    private void OnInputRotXInputEnd(string value)
    {
        if (selectedObject != null)
        {
            var rot = selectedObject.gameObject.transform.rotation.eulerAngles;
            if (float.TryParse(value, out float x))
            {
                rot.z = -x;
            }
            selectedObject.gameObject.transform.rotation = Quaternion.Euler(rot);
        }
        else
        {
            if (float.TryParse(value, out float x))
            {
                spawnRotation.z = -x;
            }
            else
            {
                // 不正入力時は元の値に戻す
                inputRotX.text = (-spawnRotation.z).ToString("F2");
            }
        }
    }

    private void OnInputRotYInputEnd(string value)
    {
        if (selectedObject != null)
        {
            var rot = selectedObject.gameObject.transform.rotation.eulerAngles;
            if (float.TryParse(value, out float y))
            {
                rot.x = y;
            }
            selectedObject.gameObject.transform.rotation = Quaternion.Euler(rot);
        }
        else
        {
            if (float.TryParse(value, out float y))
            {
                spawnRotation.x = y;
            }
            else
            {
                // 不正入力時は元の値に戻す
                inputRotY.text = spawnRotation.x.ToString("F2");
            }
        }
    }

    private void OnInputRotZInputEnd(string value)
    {
        if (selectedObject != null)
        {
            var rot = selectedObject.gameObject.transform.rotation.eulerAngles;
            if (float.TryParse(value, out float z))
            {
                rot.y = -z;
            }
            selectedObject.gameObject.transform.rotation = Quaternion.Euler(rot);
        }
        else
        {
            if (float.TryParse(value, out float z))
            {
                spawnRotation.y = -z;
            }
            else
            {
                // 不正入力時は元の値に戻す
                inputRotZ.text = (-spawnRotation.y).ToString("F2");
            }
        }
    }

    private void OnInputScaleXInputEnd(string value)
    {
        if (selectedObject != null)
        {
            var scale = selectedObject.gameObject.transform.localScale;
            if (float.TryParse(value, out float x))
            {
                scale.z = x;
            }
            selectedObject.gameObject.transform.localScale = scale;
        }
        else
        {
            if (float.TryParse(value, out float x))
            {
                spawnScale.z = x;
            }
            else
            {
                // 不正入力時は元の値に戻す
                inputScaleX.text = spawnScale.z.ToString("F2");
            }
        }
    }

    private void OnInputScaleYInputEnd(string value)
    {
        if (selectedObject != null)
        {
            var scale = selectedObject.gameObject.transform.localScale;
            if (float.TryParse(value, out float y))
            {
                scale.x = y;
            }
            selectedObject.gameObject.transform.localScale = scale;
        }
        else
        {
            if (float.TryParse(value, out float y))
            {
                spawnScale.x = y;
            }
            else
            {
                // 不正入力時は元の値に戻す
                inputScaleY.text = spawnScale.x.ToString("F2");
            }
        }
    }

    private void OnInputScaleZInputEnd(string value)
    {
        if (selectedObject != null)
        {
            var scale = selectedObject.gameObject.transform.localScale;
            if (float.TryParse(value, out float z))
            {
                scale.y = z;
            }
            selectedObject.gameObject.transform.localScale = scale;
        }
        else
        {
            if (float.TryParse(value, out float z))
            {
                spawnScale.y = z;
            }
            else
            {
                // 不正入力時は元の値に戻す
                inputScaleZ.text = spawnScale.y.ToString("F2");
            }
        }
    }
}
