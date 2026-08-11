// UnitySensors の PointCloudVisualizer<PointXYZ> (深度カメラ点群) 用の
// URP 対応シェーダ。構成は PointCloudXYZI.shader と同じで、点の色だけ
// センサ原点からの距離で近色→遠色に変える (深度の見た目づけ)。
Shader "Simulator/PointCloudXYZ"
{
    Properties
    {
        _PointSize ("Point Size", Range(0.001, 0.5)) = 0.02
        _NearColor ("Near Color", Color) = (1.0, 0.55, 0.1, 1.0)
        _FarColor ("Far Color", Color) = (0.15, 0.3, 1.0, 1.0)
        _ColorRange ("Color Range [m]", Float) = 10.0
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

            StructuredBuffer<float3> PointsBuffer;
            float4x4 LocalToWorldMatrix;
            float _PointSize;
            fixed4 _NearColor;
            fixed4 _FarColor;
            float _ColorRange;

            struct v2f
            {
                float4 pos : SV_POSITION;
                fixed4 col : COLOR;
            };

            v2f vert(float4 vertex : POSITION, uint id : SV_InstanceID)
            {
                v2f o;
                float3 p = PointsBuffer[id];
                if (dot(p, p) < 1e-12)
                {
                    o.pos = float4(0, 0, -2, 1);
                    o.col = 0;
                    return o;
                }
                float3 center = mul(LocalToWorldMatrix, float4(p, 1)).xyz;
                float3 right = float3(UNITY_MATRIX_V[0].x, UNITY_MATRIX_V[0].y, UNITY_MATRIX_V[0].z);
                float3 up = float3(UNITY_MATRIX_V[1].x, UNITY_MATRIX_V[1].y, UNITY_MATRIX_V[1].z);
                float3 world = center + (vertex.x * right + vertex.y * up) * _PointSize;
                o.pos = mul(UNITY_MATRIX_VP, float4(world, 1));
                o.col = lerp(_NearColor, _FarColor, saturate(length(p) / max(_ColorRange, 1e-3)));
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
