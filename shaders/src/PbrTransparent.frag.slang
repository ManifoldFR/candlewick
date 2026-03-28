#define HAS_WBOIT

import tone_mapping;
import pbr_lighting;
import config;

[vk::binding(0, 3)] ConstantBuffer<PbrMaterial> materialBlock;
[vk::binding(1, 3)] ConstantBuffer<LightBlock>  light;

[vk::binding(0, 2)] Sampler2D baseColorTex;

struct FSOutput {
    float4 accum  : SV_Target0; // R16G16B16A16 accumulation
    float  reveal : SV_Target1; // R8 revealage
};

[shader("fragment")]
FSOutput main([vk::location(0)] float3 fragViewPos,
              [vk::location(1)] float3 fragViewNormal,
              [vk::location(2)] float3 fragLightPos,
              [vk::location(6)] float2 fragTexCoord,
              float4 fragCoord : SV_Position) {
    float3 normal = normalize(fragViewNormal);
    float3 V = normalize(-fragViewPos);

    PbrMaterial mat = materialBlock;
    mat.baseColor *= baseColorTex.Sample(fragTexCoord);

    float3 Lo = float3(0);
    for (uint i = 0; i < uint(light.numLights); i++) {
        float3 lightDir = normalize(-light.direction[i]);
        Lo += calculatePbrLighting(normal, V, lightDir, mat,
                                   light.color[i], light.intensity[i]);
    }

    float3 ambient = float3(0.03) * mat.baseColor.rgb * mat.ao;
    float3 color = ambient + Lo;

    color = uncharted2ToneMapping(color);
    color = pow(color, float3(1.0 / 2.2));

    float alpha = mat.baseColor.a;
    const float z = fragCoord.z;
    float w1 = alpha * 10.0 + 0.01;
    float weight = clamp(w1 * (1.0 - z * 0.3), 1e-2, 1e3);

    FSOutput output;
    output.accum  = float4(color * alpha * weight, alpha * weight);
    output.reveal = alpha;
    return output;
}
