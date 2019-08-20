// Inputs
uniform vec3 eyePosition; // object space
uniform float timeVal;
uniform float scale;  // the amount to scale the noise texture by
uniform float scroll; // the amount by which to scroll the noise
uniform float noise;  // the noise perturb as a factor of the time

// Outputs to Fragment shader
varying vec3 noiseCoord;
varying vec4 projectionCoord;
varying vec3 eyeDir;
varying vec3 oNormal;

// Vertex program for fresnel reflections / refractions
void main()
{
  gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex;
  // Projective texture coordinates, adjust for mapping
  mat4 scalemat = mat4(0.5, 0.0, 0.0, 0.0,
                         0.0, -0.5, 0.0, 0.0,
                         0.0, 0.0, 0.5, 0.0,
                         0.5, 0.5, 0.5, 1.0);
  projectionCoord = scalemat * gl_Position;

  // Noise map coords
  noiseCoord.xy = (gl_MultiTexCoord0.xy + (timeVal * scroll)) * scale;
  noiseCoord.z = noise * timeVal;

  eyeDir = normalize(gl_Vertex.xyz - eyePosition);
  oNormal = gl_Normal.rgb;
}

