#version 130

uniform sampler2D reflectMap;

varying vec2 oUV0;
void main(void)
{
    gl_FragColor = texture2D(reflectMap, oUV0);
}
//
//uniform vec4 tintColour;
//uniform float noiseScale;
//uniform float fresnelBias;
//uniform float fresnelScale;
//uniform float fresnelPower;
////uniform sampler2D noiseMap;
//uniform sampler2D reflectMap;
////uniform sampler2D refractMap;
//
//varying vec3 noiseCoord;
//varying vec4 projectionCoord;
//varying vec3 eyeDir;
//varying vec3 oNormal;
//
//varying vec4 fragColour;
//
//// Fragment program for distorting a texture using a 3D noise texture
//void main(void)
//{
//	// Do the tex projection manually so we can distort _after_
//	vec2 final = projectionCoord.xy / projectionCoord.w;
//
//	// Noise
//	//vec3 noiseNormal = (texture(noiseMap, (noiseCoord.xy / 5.0)).rgb - 0.5).rbg * noiseScale;
//	//final += noiseNormal.xz;
//
//	// Fresnel
//	//normal = normalize(normal + noiseNormal.xz);
//	//float fresnel = fresnelBias + fresnelScale * pow(1.0 + dot(eyeDir, oNormal), fresnelPower);
//
//	// Reflection / refraction
//	vec4 reflectionColour = texture(reflectMap, final);
//	//vec4 refractionColour = texture(refractMap, final) + tintColour;
//
//	// Final colour
//	//fragColour = mix(refractionColour, reflectionColour, fresnel);
//	fragColour = reflectionColour;
//}
