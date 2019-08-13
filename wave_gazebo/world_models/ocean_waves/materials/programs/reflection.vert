varying vec4 color;

void main(void)
{
    color = gl_Vertex;
    gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex;
}
