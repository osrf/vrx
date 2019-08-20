// Inputs
uniform vec4 tintColour;
uniform float noiseScale;
uniform float fresnelBias;
uniform float fresnelScale;
uniform float fresnelPower;
uniform sampler2D noiseMap;
uniform sampler2D reflectMap;
uniform sampler2D refractMap;

// Inputs from Vertex shader
varying vec3 noiseCoord;
varying vec4 projectionCoord;
varying vec3 eyeDir;
varying vec3 oNormal;

// Fragment program for distorting a texture using a 3D noise texture
void main()
{
  // Do the tex projection manually so we can distort _after_
  vec2 final = projectionCoord.xy / projectionCoord.w;

  // Reflection / refraction
  vec4 reflectionColour = texture(reflectMap, final);
  vec4 refractionColour = texture(refractMap, final) + tintColour;

  // Final colour
  gl_FragColor = mix(refractionColour, reflectionColour, 0.5);
}

