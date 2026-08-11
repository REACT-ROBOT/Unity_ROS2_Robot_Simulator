// UnitySensors の PointCloudVisualizer<PointXYZI> 用の URP 対応シェーダ。
// パッケージ同梱 (Samples~) のものはビルトイン RP のサーフェスシェーダで
// URP ではマゼンタになるため、SRPDefaultUnlit で描ける素の vert/frag に
// 置き換えている。バッファのレイアウト (float3 + intensity) と
// PointsBuffer / LocalToWorldMatrix のプロパティ名は Visualizer 側と同じ。
// クアッドは軸固定ではなくカメラに正対させる。原点 (未ヒット点) は棄てる。
Shader "Simulator/PointCloudXYZI"
{
    Properties
    {
        _PointSize ("Point Size", Range(0.001, 0.5)) = 0.05
        _Color ("Color", Color) = (0.3, 1.0, 0.35, 1.0)
    }
    SubShader
    {
        Tags { "RenderType" = "Opaque" }
        Cull Off
        ZWrite Off

        Pass
        {
            CGPROGRAM
            #pragma vertex vert
            #pragma fragment frag
            #pragma target 4.5
            #include "UnityCG.cginc"

            struct Point
            {
                float3 position;
                float intensity;
            };
            StructuredBuffer<Point> PointsBuffer;
            float4x4 LocalToWorldMatrix;
            float _PointSize;
            fixed4 _Color;

            struct v2f
            {
                float4 pos : SV_POSITION;
                fixed4 col : COLOR;
            };

            v2f vert(float4 vertex : POSITION, uint id : SV_InstanceID)
            {
                v2f o;
                Point pt = PointsBuffer[id];
                if (dot(pt.position, pt.position) < 1e-12)
                {
                    // 未ヒット点 (原点) はクリップ範囲外へ飛ばして描かない
                    o.pos = float4(0, 0, -2, 1);
                    o.col = 0;
                    return o;
                }
                float3 center = mul(LocalToWorldMatrix, float4(pt.position, 1)).xyz;
                float3 right = float3(UNITY_MATRIX_V[0].x, UNITY_MATRIX_V[0].y, UNITY_MATRIX_V[0].z);
                float3 up = float3(UNITY_MATRIX_V[1].x, UNITY_MATRIX_V[1].y, UNITY_MATRIX_V[1].z);
                float3 world = center + (vertex.x * right + vertex.y * up) * _PointSize;
                o.pos = mul(UNITY_MATRIX_VP, float4(world, 1));
                o.col = _Color;
                return o;
            }

            fixed4 frag(v2f i) : SV_Target
            {
                return i.col;
            }
            ENDCG
        }
    }
}
