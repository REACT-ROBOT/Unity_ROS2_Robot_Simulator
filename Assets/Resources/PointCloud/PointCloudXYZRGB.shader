// UnitySensors の PointCloudVisualizer<PointXYZRGB> (RGBD 点群) 用の
// URP 対応シェーダ。構成は PointCloudXYZI.shader と同じ。色は
// PointXYZRGB の r,g,b,a バイト列 (リトルエンディアンで下位から r) を
// uint から展開して使う。
Shader "Simulator/PointCloudXYZRGB"
{
    Properties
    {
        _PointSize ("Point Size", Range(0.001, 0.5)) = 0.02
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
                uint rgba;
            };
            StructuredBuffer<Point> PointsBuffer;
            float4x4 LocalToWorldMatrix;
            float _PointSize;

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
                    o.pos = float4(0, 0, -2, 1);
                    o.col = 0;
                    return o;
                }
                float3 center = mul(LocalToWorldMatrix, float4(pt.position, 1)).xyz;
                float3 right = float3(UNITY_MATRIX_V[0].x, UNITY_MATRIX_V[0].y, UNITY_MATRIX_V[0].z);
                float3 up = float3(UNITY_MATRIX_V[1].x, UNITY_MATRIX_V[1].y, UNITY_MATRIX_V[1].z);
                float3 world = center + (vertex.x * right + vertex.y * up) * _PointSize;
                o.pos = mul(UNITY_MATRIX_VP, float4(world, 1));
                uint c = pt.rgba;
                o.col = fixed4((c & 255) / 255.0, (c >> 8 & 255) / 255.0, (c >> 16 & 255) / 255.0, 1.0);
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
