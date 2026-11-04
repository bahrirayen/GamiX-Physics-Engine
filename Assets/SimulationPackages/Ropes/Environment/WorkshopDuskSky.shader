Shader "GamiX/Workshop Dusk Sky"
{
    Properties
    {
        _ZenithColor ("Zenith", Color) = (0.08, 0.15, 0.28, 1)
        _HorizonColor ("Warm Horizon", Color) = (0.62, 0.30, 0.14, 1)
        _GroundColor ("Lower Sky", Color) = (0.16, 0.20, 0.27, 1)
        _SunColor ("Sun Glow", Color) = (1.0, 0.62, 0.32, 1)
        _SunDirection ("Sun Direction", Vector) = (0.47, 0.38, -0.81, 0)
    }

    SubShader
    {
        Tags { "Queue"="Background" "RenderType"="Background" "PreviewType"="Skybox" }
        Cull Off
        ZWrite Off

        Pass
        {
            HLSLPROGRAM
            #pragma vertex vert
            #pragma fragment frag
            #pragma target 3.0
            #include "UnityCG.cginc"

            half4 _ZenithColor;
            half4 _HorizonColor;
            half4 _GroundColor;
            half4 _SunColor;
            float4 _SunDirection;

            struct Attributes
            {
                float4 positionOS : POSITION;
            };

            struct Varyings
            {
                float4 positionCS : SV_POSITION;
                float3 direction : TEXCOORD0;
            };

            Varyings vert(Attributes input)
            {
                Varyings output;
                output.positionCS = UnityObjectToClipPos(input.positionOS);
#if defined(UNITY_REVERSED_Z)
                output.positionCS.z = 0.0;
#else
                output.positionCS.z = output.positionCS.w;
#endif
                output.direction = input.positionOS.xyz;
                return output;
            }

            half4 frag(Varyings input) : SV_Target
            {
                float3 direction = normalize(input.direction);
                float elevation = direction.y;

                float lowerBlend = smoothstep(-0.34, 0.035, elevation);
                float upperBlend = smoothstep(-0.015, 0.62, elevation);
                half3 sky = lerp(_GroundColor.rgb, _HorizonColor.rgb, lowerBlend);
                sky = lerp(sky, _ZenithColor.rgb, upperBlend);

                float horizonGlow = exp(-pow((elevation - 0.045) / 0.15, 2.0));
                sky += _SunColor.rgb * horizonGlow * 0.17;

                float sunDot = dot(direction, normalize(_SunDirection.xyz));
                float sunDisk = smoothstep(0.99962, 0.99986, sunDot);
                float sunHalo = pow(saturate(sunDot), 96.0) * 0.16;
                sky = lerp(sky, _SunColor.rgb * 1.45, sunDisk);
                sky += _SunColor.rgb * sunHalo;
                return half4(sky, 1.0);
            }
            ENDHLSL
        }
    }
    Fallback Off
}
