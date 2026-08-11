using UnityEngine;
using UnitySensors.Sensor;
using UnitySensors.Sensor.Camera;
using UnitySensors.Utils.PointCloud;
using UnitySensors.Visualization.Sensor;

/// <summary>
/// UnitySensors 同梱の可視化コンポーネント (PointCloudVisualizer / TextureVisualizer)
/// をランタイムで取り付けるための共有アセットと補助 API。
/// </summary>
/// <remarks>
/// パッケージ同梱の点群マテリアルは Samples~ 内のビルトイン RP 用サーフェス
/// シェーダで、本プロジェクト (URP) では使えない。そこで URP 対応の同レイアウト
/// シェーダ (Assets/Resources/PointCloud/) からマテリアルと PointUtilitiesSO を
/// 実行時に組み立てて全可視化で共有する。
///
/// 点群は専用レイヤー (SensorVisualization = 30) に描く。カメラ系センサの
/// Camera は SimulationControl のセンサ組み立て時にこのレイヤーを cullingMask
/// から外すので、可視化を有効にしても ROS へ配信される画像には写り込まない。
/// </remarks>
public static class SensorVisualization
{
    /// <summary>点群の描画専用レイヤー (TagManager の SensorVisualization)。</summary>
    public const int Layer = 30;

    static PointUtilitiesSO s_PointUtilities;

    static PointUtilitiesSO GetPointUtilities()
    {
        if (s_PointUtilities == null)
        {
            s_PointUtilities = ScriptableObject.CreateInstance<PointUtilitiesSO>();
            s_PointUtilities.Configure(
                LoadPointMaterial("PointCloud/PointCloudXYZ"),
                LoadPointMaterial("PointCloud/PointCloudXYZI"),
                LoadPointMaterial("PointCloud/PointCloudXYZRGB"));
        }
        return s_PointUtilities;
    }

    static Material LoadPointMaterial(string resourcePath)
    {
        Shader shader = Resources.Load<Shader>(resourcePath);
        if (shader == null)
        {
            Debug.LogError($"[SensorVisualization] shader not found in Resources: {resourcePath}");
            return null;
        }
        return new Material(shader);
    }

    /// <summary>センサカメラから点群レイヤーを外す (画像トピックへの写り込み防止)。</summary>
    public static void ExcludeFromCamera(Camera camera)
    {
        if (camera != null)
        {
            camera.cullingMask &= ~(1 << Layer);
        }
    }

    /// <summary>
    /// 点群系の可視化コンポーネントをセンサへ取り付ける。画像系
    /// (ImageTexture0/1) は表示先の RawImage が要るのでパネル側で
    /// TextureVisualizer を直接組む。
    /// </summary>
    public static Component AttachPointCloud(SimulationControl.GuiSensorInfo info)
    {
        GameObject target = info.sensor.gameObject;
        switch (info.kind)
        {
            case SimulationControl.GuiSensorVizKind.PointCloudXYZI:
            {
                var visualizer = target.AddComponent<LiDARPointCloudVisualizer>();
                visualizer.Configure(info.sensor, GetPointUtilities(), Layer);
                return visualizer;
            }
            case SimulationControl.GuiSensorVizKind.PointCloudXYZ:
            {
                var visualizer = target.AddComponent<DepthCameraPointCloudVisualizer>();
                visualizer.Configure(info.sensor, GetPointUtilities(), Layer);
                return visualizer;
            }
            case SimulationControl.GuiSensorVizKind.PointCloudXYZRGB:
            {
                var visualizer = target.AddComponent<RGBDCameraPointCloudVisualizer>();
                visualizer.Configure(info.sensor, GetPointUtilities(), Layer);
                return visualizer;
            }
            default:
                return null;
        }
    }

    /// <summary>取り付け済みの点群可視化を探す (パネル再構築時のトグル状態復元用)。</summary>
    public static Component FindAttachedPointCloud(SimulationControl.GuiSensorInfo info)
    {
        GameObject target = info.sensor != null ? info.sensor.gameObject : null;
        if (target == null)
        {
            return null;
        }
        switch (info.kind)
        {
            case SimulationControl.GuiSensorVizKind.PointCloudXYZI:
                return target.GetComponent<LiDARPointCloudVisualizer>();
            case SimulationControl.GuiSensorVizKind.PointCloudXYZ:
                return target.GetComponent<DepthCameraPointCloudVisualizer>();
            case SimulationControl.GuiSensorVizKind.PointCloudXYZRGB:
                return target.GetComponent<RGBDCameraPointCloudVisualizer>();
            default:
                return null;
        }
    }

    /// <summary>
    /// 点群可視化を外す。深度/RGBD の点群変換ジョブは可視化のためだけに回して
    /// いた (シミュレータは PointCloud2 を配信しない) ので、ここで止める。
    /// </summary>
    public static void DetachPointCloud(SimulationControl.GuiSensorInfo info, Component visualizer)
    {
        if (visualizer != null)
        {
            Object.Destroy(visualizer);
        }
        if (info.kind == SimulationControl.GuiSensorVizKind.PointCloudXYZ
            || info.kind == SimulationControl.GuiSensorVizKind.PointCloudXYZRGB)
        {
            if (info.sensor is DepthCameraSensor depth)
            {
                depth.convertToPointCloud = false;
            }
            else if (info.sensor is RGBDCameraSensor rgbd)
            {
                rgbd.convertToPointCloud = false;
            }
        }
    }
}
