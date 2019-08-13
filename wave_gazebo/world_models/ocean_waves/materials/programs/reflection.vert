varying vec4 color;

varying vec4 P;
void main(void)
{
    color = gl_Vertex;
    vec4 P = gl_Vertex;
    P.z += 10;
    gl_Position = gl_ModelViewProjectionMatrix * P;
}
