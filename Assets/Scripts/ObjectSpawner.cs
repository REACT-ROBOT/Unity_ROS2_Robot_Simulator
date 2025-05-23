using UnityEngine;
using TMPro;
using UnityEngine.UI;    // Button
using System.Collections.Generic;
using Unity.Robotics.UrdfImporter;
using UnityMeshImporter;
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

    // === 公開メソッド：ボタンから呼び出し ===
    public void SpawnCube()   => SpawnPrimitive(PrimitiveType.Cube);
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
        string filePath = UnityEditor.EditorUtility.OpenFilePanel("Select Mesh File", "", "3mf,dae,blend,bvh,3ds,ase,gltf,glb,fbx,ply,dxf,ifc,nff,smd,vta,mdl,md2,md3,pk3,mdc,md5mesh,x,q3o,q3s,raw,ac,stl,irrmesh,irr,obj,ter,mdl,hmp,ogex,ms3d,lwo,lws,lxo,csm");
        if (string.IsNullOrEmpty(filePath))
        {
            Debug.LogWarning("No file selected.");
            return;
        }

        var ob = MeshImporter.Load(filePath);
        if (ob == null)
        {
            Debug.LogError("Failed to load object from File.");
            return;
        }
        ob.name = $"Mesh_{spawnedObjects.Count}";

        // オブジェクトの直下のすべての子オブジェクトを取得
        foreach (MeshCollider meshCollider in ob.GetComponentsInChildren<MeshCollider>())
        {
            meshCollider.sharedMesh = meshCollider.gameObject.GetComponent<MeshFilter>().mesh;
        }

        ob.transform.position = spawnPosition;
        ob.transform.rotation = Quaternion.Euler(spawnRotation);
        ob.transform.localScale = spawnScale;

        AddListItem(ob);
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
        light.range     = 10f;
        light.intensity = 20f;
        go.transform.position = spawnPosition;
        AddListItem(go);
    }

    /// <summary>Spot Light を生成</summary>
    public void SpawnSpotLight()
    {
        var go = new GameObject($"SpotLight_{spawnedObjects.Count}");
        var light = go.AddComponent<Light>();
        light.type       = LightType.Spot;
        light.shadows = LightShadows.Soft;
        light.range      = 15f;
        light.intensity  = 1f;
        light.spotAngle  = 30f;
        go.transform.position = spawnPosition;
        go.transform.rotation = Quaternion.Euler(spawnRotation);
        AddListItem(go);
    }

    // リストに行アイテムを追加して、ボタンに選択イベントを登録
    private void AddListItem(GameObject go)
    {
        var item = Instantiate(listItemPrefab, listContent);
        var text = item.GetComponentInChildren<TMP_Text>();
        if (text != null) text.text = go.name;

        var btn = item.GetComponent<Button>();
        if (btn != null)
        {
            spawnedObjects.Add(new SpawnedObject { gameObject = go, button = btn });
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
