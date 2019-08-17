// varying vec4 color;
// 
// varying vec4 P;
// void main(void)
// {
//     color = gl_Vertex;
//     vec4 P = gl_Vertex;
//     //P.z += 10;
//     gl_Position = gl_ModelViewProjectionMatrix * P;
// }

//varying vec4 uv0;
//varying vec4 position;
//varying vec3 normal;
//
//uniform mat4 worldViewProjMatrix;
//uniform vec3 eyePosition; // object space
//uniform float timeVal;
//uniform float scale;  // the amount to scale the noise texture by
//uniform float scroll; // the amount by which to scroll the noise
//uniform float noise;  // the noise perturb as a factor of the time
//
//varying vec3 noiseCoord;
//varying vec4 projectionCoord;
//varying vec3 eyeDir;
//varying vec3 oNormal;
//
//// Vertex program for fresnel reflections / refractions
//void main(void)
//{
//	gl_Position = worldViewProjMatrix * position;
//	// Projective texture coordinates, adjust for mapping
//	mat4 scalemat = mat4(0.5, 0.0, 0.0, 0.0,
//                         0.0, -0.5, 0.0, 0.0,
//                         0.0, 0.0, 0.5, 0.0,
//                         0.5, 0.5, 0.5, 1.0);
//	projectionCoord = scalemat * gl_Position;
//
//	// Noise map coords
//	noiseCoord.xy = (uv0.xy + (timeVal * scroll)) * scale;
//	noiseCoord.z = noise * timeVal;
//
//	eyeDir = normalize(position.xyz - eyePosition);
//	oNormal = normal.rgb;
//}
