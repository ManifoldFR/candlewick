import config;

struct TranformBlock {
    float4x4 modelView;
    float4x4 mvp;
    float3x3 normalMatrix;
};

struct LightBlockV {
    float4x4 mvp[MAX_NUM_LIGHTS];
    int numLights;
};

[vk::binding(0, 1)] ConstantBuffer<TranformBlock> transform;
[vk::binding(1, 1)] ConstantBuffer<LightBlockV> lights;

struct VSOutput {
    [vk::location(0)] float3 fragViewPos;
    [vk::location(1)] float3 fragViewNormal;
    [vk::location(2)] float3 fragLightPos[MAX_NUM_LIGHTS];
    [vk::location(6)] float2 fragTexCoord;
    float4 position : SV_Position;
};

[shader("vertex")]
VSOutput main([vk::location(0)] float3 inPosition,
              [vk::location(1)] float3 inNormal,
              [vk::location(6)] float2 inTexCoord) {
    VSOutput output;
    float4 hp = float4(inPosition, 1.0);
    output.fragViewPos = mul(transform.modelView, hp).xyz;
    output.fragViewNormal = normalize(mul(transform.normalMatrix, inNormal));
    output.position = mul(transform.mvp, hp);
    output.fragTexCoord = inTexCoord;

    for (uint i = 0; i < uint(lights.numLights); i++) {
        float4 flps = mul(lights.mvp[i], hp);
        output.fragLightPos[i] = flps.xyz / flps.w;
    }
    return output;
}
