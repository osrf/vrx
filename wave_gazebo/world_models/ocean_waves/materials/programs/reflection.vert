#version 130
attribute vec4 position;
attribute vec2 uv0;

uniform mat4 worldViewProjMatrix;

varying vec4 projectionCoord;

void main()
{
  gl_Position = worldViewProjMatrix * position;
  // Projective texture coordinates, adjust for mapping
  mat4 scalemat = mat4(0.5, 0.0, 0.0, 0.0, 
                         0.0, -0.5, 0.0, 0.0,
                         0.0, 0.0, 0.5, 0.0,
                         0.5, 0.5, 0.5, 1.0);

  projectionCoord = scalemat * gl_Position;
}
