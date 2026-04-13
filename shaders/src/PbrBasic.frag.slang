#define HAS_SHADOW_MAPS
#define HAS_G_BUFFER
#define HAS_SSAO

import utils;
import tone_mapping;
import pbr_lighting;
import config;

struct ShadowAtlasInfo {
    int4 lightRegions[MAX_NUM_LIGHTS];
};

[vk::binding(0, 3)] ConstantBuffer<PbrMaterial>      materialBlock;
[vk::binding(1, 3)] ConstantBuffer<LightBlock>        light;
[vk::binding(2, 3)] ConstantBuffer<ShadowAtlasInfo>   shadowAtlas;

#ifdef HAS_SHADOW_MAPS
    [vk::binding(0, 2)] Sampler2DShadow shadowMap;
#endif
#ifdef HAS_SSAO
    [vk::binding(1, 2)] Sampler2D ssaoTex;
#endif

[vk::binding(2, 2)] Sampler2D baseColorTex;

struct FSOutput {
    float4 fragColor : SV_Target0;
#ifdef HAS_G_BUFFER
    float2 outNormal : SV_Target1;
    float  outDepth  : SV_Target2;
#endif
};

#ifdef HAS_SHADOW_MAPS
float calcShadowmap(int lightIndex, float NdotL, int2 atlasSize,
                    float3 fragLightPos[MAX_NUM_LIGHTS]) {
    float bias = 0.005;
    float3 lightSpacePos = fragLightPos[lightIndex];
    float2 uv;
    uv.x = 0.5 + lightSpacePos.x * 0.5;
    uv.y = 0.5 - lightSpacePos.y * 0.5;
    float depthRef = lightSpacePos.z - bias;
    if (!isCoordsInRange(float3(uv, depthRef))) {
        return 1.0;
    }

    int4 region = shadowAtlas.lightRegions[lightIndex];
    uv = (region.xy + uv * region.zw) / atlasSize;
    float2 regionMin = float2(region.xy) / atlasSize;
    float2 regionMax = float2(region.xy + region.zw) / atlasSize;

    float value = 0.0;
    const float2 offsets = 1.0 / atlasSize;
    const int halfKernel = 1;
    const float weight = 1.0 / pow(2 * halfKernel + 1, 2);
    for (int i = -halfKernel; i <= halfKernel; i++) {
        for (int j = -halfKernel; j <= halfKernel; j++) {
            float2 offUV = clamp(uv + float2(i, j) * offsets, regionMin, regionMax);
            value += weight * shadowMap.SampleCmp(offUV, depthRef);
        }
    }
    return value;
}
#endif

[shader("fragment")]
FSOutput main([vk::location(0)] float3 fragViewPos,
              [vk::location(1)] float3 fragViewNormal,
              [vk::location(2)] float3 fragLightPos[MAX_NUM_LIGHTS],
              [vk::location(6)] float2 fragTexCoord,
              float4 fragCoord : SV_Position,
              bool isFrontFacing : SV_IsFrontFace) {
    float3 normal = normalize(fragViewNormal);
    const float3 V = normalize(-fragViewPos);

#ifdef HAS_SHADOW_MAPS
    uint smW, smH;
    shadowMap.GetDimensions(smW, smH);
    const int2 atlasSize = int2(smW, smH);
#endif

    if (!isFrontFacing) {
        normal = -normal;
    }

    PbrMaterial mat = materialBlock;
    mat.baseColor *= baseColorTex.Sample(fragTexCoord);

    float3 Lo = float3(0);
    for (int i = 0; i < light.numLights; i++) {
        float3 lightDir = normalize(-light.direction[i]);
        float3 _lo = calculatePbrLighting(normal, V, lightDir, mat,
                                          light.color[i], light.intensity[i]);
#ifdef HAS_SHADOW_MAPS
        const float NdotL = max(dot(normal, lightDir), 0.0);
        _lo *= calcShadowmap(i, NdotL, atlasSize, fragLightPos);
#endif
        Lo += _lo;
    }

    float3 ambient = float3(0.1) * mat.baseColor.rgb * mat.ao;

#ifdef HAS_SSAO
    if (light.useSsao != 0) {
        uint ssaoW, ssaoH;
        ssaoTex.GetDimensions(ssaoW, ssaoH);
        float2 ssaoUV = fragCoord.xy / float2(ssaoW, ssaoH);
        ambient *= ssaoTex.Sample(ssaoUV).r;
    }
#endif

    float3 color = ambient + Lo;
    color = uncharted2ToneMapping(color);
    color = pow(color, float3(1.0 / 2.2));

    FSOutput output;
    output.fragColor = float4(color, mat.baseColor.a);
#ifdef HAS_G_BUFFER
    output.outNormal = fragViewNormal.rg;
    output.outDepth  = fragCoord.z;
#endif
    return output;
}
