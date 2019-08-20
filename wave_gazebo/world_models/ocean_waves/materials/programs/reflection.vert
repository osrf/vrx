#version 130

varying vec4 projectionCoord;

void main()
{
  gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex;
  // Projective texture coordinates, adjust for mapping
  mat4 scalemat = mat4(0.5, 0.0, 0.0, 0.0, 
                         0.0, -0.5, 0.0, 0.0,
                         0.0, 0.0, 0.5, 0.0,
                         0.5, 0.5, 0.5, 1.0);

  projectionCoord = scalemat * gl_Position;
}
